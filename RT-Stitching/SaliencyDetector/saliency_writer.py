"""
saliency_writer.py
==================
基于 PyTorch U²-Net 的显著性检测写入器。

【本版新增 canvas 闭环模式（解决两路拼接 + 实时同步）】
  C++ SeamFinder 在 warp 两路视频后，把合成画布写到共享目录：
      <dir>/canvas_in.png   两路 warped 图贴到 canvas 后的 BGR 合成图
      <dir>/canvas_in.seq   纯文本递增序列号（变化才重跑）
  本脚本 canvas 模式轮询 canvas_in.seq，变化就读 canvas_in.png 跑 U²-Net，
  把显著性写回：
      <dir>/saliency.png    8 位单通道显著性图（0~255 即 [0,1]*255）
      <dir>/saliency.seq    纯文本递增序列号
  C++ SaliencyMapReader 再读回 saliency.png。

  这样 Python 处理的永远是流水线【当前 warp 出来的真实画布】（覆盖两路相机、
  canvas 坐标系对齐），而不是一个独立的、与实时输入脱钩的视频文件。

依赖：
  pip install torch torchvision opencv-python numpy
  U²-Net 权重 u2net.pth / u2netp.pth，模型定义 u2net.py 来自官方仓库
  https://github.com/xuebinqin/U-2-Net  (model/u2net.py)

用法：
  # 【推荐】canvas 闭环：与 C++ SeamFinder 联动（先启动 RTStitcher，再启动本脚本）
  python saliency_writer.py --source canvas --out_dir ./saliency_out ^
         --model u2net.pth --model_type u2net

  # 单张图跑一次（肉眼确认 U²-Net 效果是否合理）
  python saliency_writer.py --source image --image test.jpg --once ^
         --model u2net.pth --model_type u2net --out_dir ./saliency_out

  # 旧：监视一张全景结果图（mtime 变化重跑）
  python saliency_writer.py --source panorama --image ./out/result.jpg --out_dir ./saliency_out

  # 旧：独立视频源（仅离线演示，注意：与流水线不同步）
  python saliency_writer.py --source video --video ./video0-skip10.mp4 --out_dir ./saliency_out
"""

import os
import time
import argparse

import cv2
import numpy as np

try:
    import torch
    _TORCH_OK = True
except ImportError:
    _TORCH_OK = False

# ============ 配置参数 ============
DEFAULT_OUT_DIR = "./saliency_out"
# C++ 产出（本脚本读取）
CANVAS_PNG = "canvas_in.bmp"
CANVAS_SEQ = "canvas_in.seq"
# 本脚本产出（C++ 读取）
SAL_PNG = "saliency.bmp"
SAL_SEQ = "saliency.seq"

MODEL_INPUT_SIZE = 320          # U²-Net 标准输入边长
CANVAS_POLL = 0.02              # canvas 模式轮询间隔（秒）



# ============================================================
# U2-Net 封装
# ============================================================
class U2NetSaliency:
    def __init__(self, model_path, model_type="u2netp", device=None):
        if not _TORCH_OK:
            raise RuntimeError("PyTorch 未安装，无法加载 U²-Net。pip install torch")
        try:
            if model_type == "u2netp":
                from u2net import U2NETP as Net
            else:
                from u2net import U2NET as Net
        except ImportError as e:
            raise ImportError(
                "未找到 u2net.py 模型定义。请从 https://github.com/xuebinqin/U-2-Net "
                "下载 model/u2net.py 放到本目录。原始错误：" + str(e))

        self.device = device or ("cuda" if torch.cuda.is_available() else "cpu")
        self.net = Net(3, 1)
        state = torch.load(model_path, map_location=self.device, weights_only=False)
        self.net.load_state_dict(state)
        self.net.to(self.device)
        self.net.eval()
        print(f"[SaliencyWriter] U²-Net ({model_type}) loaded on {self.device}")

    @staticmethod
    def _preprocess(bgr, size):
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
        rgb = cv2.resize(rgb, (size, size), interpolation=cv2.INTER_AREA)
        mean = np.array([0.485, 0.456, 0.406], dtype=np.float32)
        std = np.array([0.229, 0.224, 0.225], dtype=np.float32)
        rgb = (rgb - mean) / std
        chw = np.transpose(rgb, (2, 0, 1))[None, ...]
        return chw

    @staticmethod
    def _normalize(pred):
        mn, mx = float(pred.min()), float(pred.max())
        if mx - mn < 1e-6:
            return np.zeros_like(pred)
        return (pred - mn) / (mx - mn)

    def infer(self, bgr):
        h, w = bgr.shape[:2]
        x = torch.from_numpy(self._preprocess(bgr, MODEL_INPUT_SIZE)).to(self.device)
        with torch.no_grad():
            d = self.net(x)
            d0 = d[0] if isinstance(d, (tuple, list)) else d
            pred = d0[:, 0, :, :].cpu().numpy()[0]
        pred = self._normalize(pred)
        sal = cv2.resize(pred, (w, h), interpolation=cv2.INTER_LINEAR)
        return sal.astype(np.float32)


# ============================================================
# 显著性写入（原子写：先写完整 png 再更新 seq）
# ============================================================
class SaliencyFileWriter:
    def __init__(self, out_dir):
        self.out_dir = out_dir
        os.makedirs(out_dir, exist_ok=True)
        self.png_path = os.path.join(out_dir, SAL_PNG)
        self.seq_path = os.path.join(out_dir, SAL_SEQ)
        self.seq = 0

    def write(self, saliency_f32):
        self.seq += 1
        sal_8u = np.clip(saliency_f32 * 255.0, 0, 255).astype(np.uint8)

        # 1) 原子写 png：tmp(.png 扩展名) -> rename
        tmp_png = os.path.join(self.out_dir, "saliency.tmp.png")
        if not cv2.imwrite(tmp_png, sal_8u):
            raise RuntimeError(f"cv2.imwrite 失败: {tmp_png}")
        try:
            if os.path.exists(self.png_path):
                os.remove(self.png_path)
            os.rename(tmp_png, self.png_path)
        except OSError:
            cv2.imwrite(self.png_path, sal_8u)

        # 2) png 落盘后再更新 seq
        tmp_seq = self.seq_path + ".tmp"
        with open(tmp_seq, "w") as f:
            f.write(str(self.seq))
        try:
            if os.path.exists(self.seq_path):
                os.remove(self.seq_path)
            os.rename(tmp_seq, self.seq_path)
        except OSError:
            with open(self.seq_path, "w") as f:
                f.write(str(self.seq))


# ============================================================
# 运行模式
# ============================================================
def _read_seq(path):
    """读 C++ 写的 canvas_in.seq，失败/半写返回 None"""
    try:
        with open(path, "r") as f:
            s = f.read().strip()
        return int(s) if s else None
    except (OSError, ValueError):
        return None


def _clean_session(out_dir):
    """
    启动时清掉上一轮残留文件，避免 C++ 把旧 saliency.png 当新数据读
    （即“以为成功了、其实用的是上次输出”的根源）。
    """
    for name in (CANVAS_PNG, CANVAS_SEQ, SAL_PNG, SAL_SEQ,
                 "canvas_in.tmp.png", "canvas_in.seq.tmp",
                 "saliency.tmp.png", "saliency.seq.tmp"):
        p = os.path.join(out_dir, name)
        try:
            if os.path.exists(p):
                os.remove(p)
        except OSError:
            pass
    print(f"[SaliencyWriter] 已清理上一轮残留文件: {out_dir}")


def run_canvas_loop(args, writer):
    """
    闭环模式：轮询 C++ 写出的 canvas_in.seq，变化就读 canvas_in.png 跑 U²-Net。
    这是与流水线同步的正确方式（处理的是当前 warp 画布，覆盖两路相机）。
    """
    net = U2NetSaliency(args.model, args.model_type)
    if not args.no_clean:
        _clean_session(args.out_dir)
    canvas_png = os.path.join(args.out_dir, CANVAS_PNG)
    canvas_seq = os.path.join(args.out_dir, CANVAS_SEQ)
    last_seq = None
    print(f"[SaliencyWriter] canvas 闭环模式，监视 {canvas_seq}")
    print(f"[SaliencyWriter] 等待 C++ SeamFinder 写出 canvas_in.png ...")
    try:
        while True:
            seq = _read_seq(canvas_seq)
            if seq is not None and seq != last_seq:
                bgr = cv2.imread(canvas_png, cv2.IMREAD_COLOR)
                if bgr is not None and bgr.size > 0:
                    sal = net.infer(bgr)
                    writer.write(sal)
                    last_seq = seq
                    print(f"[SaliencyWriter] canvas_seq={seq} -> saliency_seq={writer.seq}")
                # bgr 读失败说明正在 rename，下次重试，不更新 last_seq
            time.sleep(CANVAS_POLL)
    except KeyboardInterrupt:
        print("[SaliencyWriter] stopped by user")


def run_image_once(args, writer):
    bgr = cv2.imread(args.image)
    if bgr is None:
        print(f"[SaliencyWriter] 无法读取图片: {args.image}")
        return
    net = U2NetSaliency(args.model, args.model_type)
    sal = net.infer(bgr)
    writer.write(sal)
    heat = cv2.applyColorMap((sal * 255).astype(np.uint8), cv2.COLORMAP_JET)
    overlay = cv2.addWeighted(bgr, 0.6, heat, 0.4, 0)
    cv2.imwrite(os.path.join(args.out_dir, "saliency_overlay.png"), overlay)
    print(f"[SaliencyWriter] 已写出 {writer.png_path} 和 saliency_overlay.png（请肉眼确认）")


def run_panorama_loop(args, writer):
    net = U2NetSaliency(args.model, args.model_type)
    last_mtime = 0.0
    print(f"[SaliencyWriter] 监视全景图 {args.image}（mtime 变化重跑）")
    try:
        while True:
            if os.path.exists(args.image):
                mtime = os.path.getmtime(args.image)
                if mtime != last_mtime:
                    bgr = cv2.imread(args.image)
                    if bgr is not None:
                        sal = net.infer(bgr)
                        writer.write(sal)
                        last_mtime = mtime
                        print(f"[SaliencyWriter] seq={writer.seq} written")
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("[SaliencyWriter] stopped by user")


def run_video_loop(args, writer):
    net = U2NetSaliency(args.model, args.model_type)
    cap = cv2.VideoCapture(args.video)
    if not cap.isOpened():
        print(f"[SaliencyWriter] 无法打开视频: {args.video}")
        return
    print(f"[SaliencyWriter] 独立视频源（注意：与流水线不同步）: {args.video}")
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                continue
            sal = net.infer(frame)
            writer.write(sal)
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("[SaliencyWriter] stopped by user")
    finally:
        cap.release()


# ============================================================
# 入口
# ============================================================
def main():
    ap = argparse.ArgumentParser(description="U²-Net 显著性写入器")
    ap.add_argument("--source", choices=["canvas", "image", "panorama", "video"],
                    default="canvas", help="检测源类型（推荐 canvas 闭环）")
    ap.add_argument("--image", default="./out/result.jpg")
    ap.add_argument("--video", default="./video0-skip10.mp4")
    ap.add_argument("--out_dir", default=DEFAULT_OUT_DIR,
                    help="共享目录（与 C++ sal_u2net_dir 一致）")
    ap.add_argument("--model", default="./u2netp.pth")
    ap.add_argument("--model_type", choices=["u2net", "u2netp"], default="u2netp")
    ap.add_argument("--once", action="store_true", help="image 模式只跑一次")
    ap.add_argument("--no_clean", action="store_true",
                    help="canvas 模式启动时不清理上一轮残留文件")
    ap.add_argument("--config", default=None,
                    help="可选：从 config.yaml 读取 saliency.u2net_dir 作为 out_dir，"
                         "按相对 config 目录解析，避免与 C++ 端路径不一致")
    args = ap.parse_args()

    # 若给了 --config，则用 config 里的 u2net_dir 覆盖 out_dir（与 C++ 同一规则：
    # 相对路径锚定到 config.yaml 所在目录），彻底消除两端路径不一致的问题。
    if args.config:
        try:
            import yaml
            cfg_path = os.path.abspath(args.config)
            cfg_dir = os.path.dirname(cfg_path)
            with open(cfg_path, "r", encoding="utf-8") as f:
                cfg = yaml.safe_load(f)
            u2dir = cfg["stitching"]["saliency"]["u2net_dir"]
            if not os.path.isabs(u2dir):
                u2dir = os.path.normpath(os.path.join(cfg_dir, u2dir))
            args.out_dir = u2dir
            print(f"[SaliencyWriter] 从 config 读取 out_dir = {args.out_dir}")
        except Exception as e:
            print(f"[SaliencyWriter] 读取 --config 失败（回退到 --out_dir）：{e}")

    writer = SaliencyFileWriter(args.out_dir)

    if args.source == "image" or args.once:
        run_image_once(args, writer)
    elif args.source == "canvas":
        run_canvas_loop(args, writer)
    elif args.source == "panorama":
        run_panorama_loop(args, writer)
    elif args.source == "video":
        run_video_loop(args, writer)


if __name__ == "__main__":
    main()
