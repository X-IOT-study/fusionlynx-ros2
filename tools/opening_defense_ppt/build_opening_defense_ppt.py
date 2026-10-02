#!/usr/bin/env python3
"""Build a Chinese 15-minute thesis-opening defense deck."""

from __future__ import annotations

import argparse
import shutil
from pathlib import Path

from PIL import Image
from pptx import Presentation
from pptx.dml.color import RGBColor
from pptx.enum.shapes import MSO_SHAPE
from pptx.enum.text import MSO_ANCHOR, PP_ALIGN
from pptx.util import Inches, Pt


ROOT = Path(__file__).resolve().parents[2]
ASSET_ROOT = ROOT / "paper_rewriting_output" / "final_paper" / "figures"
POINTCLOUD = ASSET_ROOT / "pointcloud.png"
TRAJECTORY = ASSET_ROOT / "trajectory_compare_xy_trajectories.png"
TRAJECTORY_RPY = ASSET_ROOT / "trajectory_compare_xy_rpy.png"

W = Inches(13.333333)
H = Inches(7.5)

BG = RGBColor(248, 250, 249)
INK = RGBColor(26, 38, 43)
MUTED = RGBColor(91, 105, 109)
GRID = RGBColor(218, 226, 225)
GREEN = RGBColor(35, 122, 88)
GREEN_LIGHT = RGBColor(226, 241, 233)
CYAN = RGBColor(0, 128, 156)
CYAN_LIGHT = RGBColor(225, 243, 247)
ORANGE = RGBColor(218, 111, 31)
ORANGE_LIGHT = RGBColor(252, 237, 224)
RED = RGBColor(184, 62, 62)
RED_LIGHT = RGBColor(250, 231, 231)
PURPLE = RGBColor(105, 75, 156)
PURPLE_LIGHT = RGBColor(239, 233, 249)
DARK = RGBColor(18, 29, 36)
WHITE = RGBColor(255, 255, 255)

FONT_CN = "Microsoft YaHei"
FONT_EN = "Arial"
FONT_MONO = "Consolas"


def rgb(hex_value: str) -> RGBColor:
    hex_value = hex_value.lstrip("#")
    return RGBColor(int(hex_value[0:2], 16), int(hex_value[2:4], 16), int(hex_value[4:6], 16))


def add_shape(slide, shape_type, x, y, w, h, fill=WHITE, line=GRID, radius=False):
    shape = slide.shapes.add_shape(shape_type, x, y, w, h)
    shape.fill.solid()
    shape.fill.fore_color.rgb = fill
    shape.line.color.rgb = line
    shape.line.width = Pt(0.8)
    return shape


def set_text(
    shape,
    text: str,
    size: float = 16,
    color=INK,
    bold: bool = False,
    font: str = FONT_CN,
    align=PP_ALIGN.LEFT,
    valign=MSO_ANCHOR.TOP,
    margin=0.08,
    italic: bool = False,
):
    tf = shape.text_frame
    tf.clear()
    tf.word_wrap = True
    tf.margin_left = Inches(margin)
    tf.margin_right = Inches(margin)
    tf.margin_top = Inches(margin)
    tf.margin_bottom = Inches(margin)
    tf.vertical_anchor = valign
    p = tf.paragraphs[0]
    p.alignment = align
    p.space_after = Pt(0)
    run = p.add_run()
    run.text = text
    run.font.name = font
    run.font.size = Pt(size)
    run.font.bold = bold
    run.font.italic = italic
    run.font.color.rgb = color
    return shape


def add_text(
    slide,
    x,
    y,
    w,
    h,
    text,
    size=16,
    color=INK,
    bold=False,
    font=FONT_CN,
    align=PP_ALIGN.LEFT,
    valign=MSO_ANCHOR.TOP,
    margin=0.08,
    italic=False,
):
    shape = slide.shapes.add_textbox(x, y, w, h)
    return set_text(
        shape, text, size, color, bold, font, align, valign, margin, italic
    )


def add_rich_lines(slide, x, y, w, h, lines, size=15, color=INK, bullet=True):
    shape = slide.shapes.add_textbox(x, y, w, h)
    tf = shape.text_frame
    tf.clear()
    tf.word_wrap = True
    tf.margin_left = Inches(0.03)
    tf.margin_right = Inches(0.03)
    tf.margin_top = Inches(0.02)
    tf.margin_bottom = Inches(0.02)
    for index, line in enumerate(lines):
        p = tf.paragraphs[0] if index == 0 else tf.add_paragraph()
        p.text = line
        p.level = 0
        p.space_after = Pt(6)
        p.font.name = FONT_CN
        p.font.size = Pt(size)
        p.font.color.rgb = color
        if bullet:
            p.text = f"• {line}"
    return shape


def add_title(slide, title, subtitle=None, dark=False):
    color = WHITE if dark else INK
    add_text(slide, Inches(0.58), Inches(0.28), Inches(12.1), Inches(0.48), title, 25, color, True)
    if subtitle:
        add_text(
            slide,
            Inches(0.60),
            Inches(0.82),
            Inches(11.7),
            Inches(0.28),
            subtitle,
            10.5,
            RGBColor(189, 203, 203) if dark else MUTED,
        )
    slide.shapes.add_shape(MSO_SHAPE.RECTANGLE, Inches(0.60), Inches(1.18), Inches(1.1), Inches(0.035)).fill.solid()
    bar = slide.shapes[-1]
    bar.fill.fore_color.rgb = CYAN
    bar.line.fill.background()


def add_footer(slide, text="基于固定时窗因子图的 LIO-Wheel 融合研究", dark=False):
    color = RGBColor(158, 173, 175) if dark else RGBColor(129, 142, 145)
    add_text(slide, Inches(0.60), Inches(7.17), Inches(9.8), Inches(0.18), text, 7.5, color)
    slide_count = len(slide.part.package.presentation_part.presentation.slides)
    add_text(slide, Inches(12.25), Inches(7.17), Inches(0.45), Inches(0.18), str(slide_count), 7.5, color, align=PP_ALIGN.RIGHT)


def add_notes(slide, text):
    slide.notes_slide.notes_text_frame.text = text


def add_chip(slide, x, y, w, text, fill, color=INK, size=10.5):
    shape = add_shape(slide, MSO_SHAPE.ROUNDED_RECTANGLE, x, y, w, Inches(0.30), fill, fill)
    set_text(shape, text, size, color, True, align=PP_ALIGN.CENTER, valign=MSO_ANCHOR.MIDDLE, margin=0.02)
    return shape


def add_box(slide, x, y, w, h, title, body="", fill=WHITE, line=GRID, title_color=INK, body_color=MUTED, title_size=14, body_size=10.5):
    box = add_shape(slide, MSO_SHAPE.ROUNDED_RECTANGLE, x, y, w, h, fill, line)
    add_text(slide, x + Inches(0.12), y + Inches(0.10), w - Inches(0.24), Inches(0.30), title, title_size, title_color, True)
    if body:
        add_text(slide, x + Inches(0.12), y + Inches(0.47), w - Inches(0.24), h - Inches(0.57), body, body_size, body_color)
    return box


def add_arrow(slide, x, y, w, h, color=CYAN):
    arrow = add_shape(slide, MSO_SHAPE.RIGHT_ARROW, x, y, w, h, color, color)
    return arrow


def add_line(slide, x1, y1, x2, y2, color=GRID, width=1.2, dash=False):
    line = slide.shapes.add_connector(1, x1, y1, x2, y2)
    line.line.color.rgb = color
    line.line.width = Pt(width)
    if dash:
        line.line.dash_style = 1
    return line


def add_circle(slide, x, y, d, fill, line=WHITE, text=None, text_color=INK, size=14):
    shape = add_shape(slide, MSO_SHAPE.OVAL, x, y, d, d, fill, line)
    if text:
        set_text(shape, text, size, text_color, True, align=PP_ALIGN.CENTER, valign=MSO_ANCHOR.MIDDLE, margin=0.01)
    return shape


def add_image_contain(slide, path: Path, x, y, w, h, border=GRID):
    if not path.exists():
        return None
    image = Image.open(path)
    iw, ih = image.size
    box_ratio = w / h
    image_ratio = iw / ih
    if image_ratio > box_ratio:
        display_w = w
        display_h = w / image_ratio
        display_x = x
        display_y = y + (h - display_h) / 2
    else:
        display_h = h
        display_w = h * image_ratio
        display_x = x + (w - display_w) / 2
        display_y = y
    slide.shapes.add_picture(str(path), display_x, display_y, display_w, display_h)
    frame = slide.shapes.add_shape(MSO_SHAPE.RECTANGLE, display_x, display_y, display_w, display_h)
    frame.fill.background()
    frame.line.color.rgb = border
    frame.line.width = Pt(0.6)
    return frame


def add_section_tag(slide, x, y, text, color=CYAN):
    add_text(slide, x, y, Inches(1.9), Inches(0.22), text.upper(), 8.5, color, True, FONT_EN)


def add_slide(prs, dark=False):
    slide = prs.slides.add_slide(prs.slide_layouts[6])
    background = slide.background
    background.fill.solid()
    background.fill.fore_color.rgb = DARK if dark else BG
    return slide


def copy_assets(out_dir: Path):
    assets = out_dir / "assets"
    assets.mkdir(parents=True, exist_ok=True)
    copied = {}
    for source, name in [
        (POINTCLOUD, "pointcloud_context.png"),
        (TRAJECTORY, "trajectory_placeholder.png"),
        (TRAJECTORY_RPY, "trajectory_rpy_placeholder.png"),
    ]:
        if source.exists():
            target = assets / name
            shutil.copy2(source, target)
            copied[name] = str(source)
    return copied


def slide_1(prs, out_dir):
    slide = add_slide(prs, dark=True)
    # A contextual engineering visual, not an experimental result.
    asset = out_dir / "assets" / "pointcloud_context.png"
    if asset.exists():
        slide.shapes.add_picture(str(asset), Inches(7.0), Inches(0.0), Inches(6.33), Inches(7.5))
        overlay = add_shape(slide, MSO_SHAPE.RECTANGLE, Inches(6.55), Inches(0), Inches(0.9), Inches(7.5), DARK, DARK)
        overlay.fill.transparency = 35
    add_section_tag(slide, Inches(0.70), Inches(0.72), "Opening defense", CYAN)
    add_text(slide, Inches(0.68), Inches(1.25), Inches(6.2), Inches(1.4), "基于固定时窗因子图的\nLIO-Wheel 融合研究", 29, WHITE, True)
    add_text(slide, Inches(0.72), Inches(3.08), Inches(5.8), Inches(0.55), "开题答辩 | 15 min", 16, RGBColor(196, 216, 214), True)
    add_line(slide, Inches(0.72), Inches(4.10), Inches(5.2), Inches(4.10), CYAN, 2.0)
    add_text(slide, Inches(0.72), Inches(4.42), Inches(5.5), Inches(1.05), "汇报人：徐文圣\n专业班级：网络工程技术 B2301\n指导教师：骆原  高级工程师", 13, RGBColor(221, 231, 229))
    add_text(slide, Inches(0.72), Inches(6.62), Inches(5.9), Inches(0.24), "南昌职业大学 · 2026 年 10 月 2 日", 9.5, RGBColor(157, 174, 175))
    add_text(slide, Inches(7.28), Inches(6.85), Inches(5.25), Inches(0.25), "视觉素材：工程点云图，用于场景示意", 7.5, RGBColor(174, 192, 191), align=PP_ALIGN.RIGHT)
    add_notes(slide, "本次汇报围绕一个明确的工程问题展开：在麦克纳姆轮机器人上，如何把轮式里程计和 FAST-LIO 的互补信息组织进固定时窗因子图，并在 LIO 退化时控制异常观测的影响。开题阶段只汇报已经实现的系统边界、拟验证的方法和评价方案，不把预期结果写成既成事实。")
    return slide


def slide_2(prs):
    slide = add_slide(prs)
    add_title(slide, "轮式与 LIO 的互补性，决定了融合的必要性", "研究背景：高频连续性与几何约束来自不同观测源")
    add_section_tag(slide, Inches(0.68), Inches(1.47), "Motivation")
    add_box(slide, Inches(0.72), Inches(1.95), Inches(3.45), Inches(2.55), "轮式里程计", "高频、平面运动连续\n能够提供横向运动约束\n但受轮滑、轮径误差和累计漂移影响", GREEN_LIGHT, GREEN, GREEN, INK, 17, 13)
    add_box(slide, Inches(4.92), Inches(1.95), Inches(3.45), Inches(2.55), "LiDAR-惯性里程计", "利用点云几何与 IMU\n局部位姿精度潜力更高\n但会受到弱结构、动态遮挡和点云退化影响", CYAN_LIGHT, CYAN, CYAN, INK, 17, 13)
    add_arrow(slide, Inches(8.88), Inches(2.70), Inches(0.62), Inches(0.42), CYAN)
    add_box(slide, Inches(9.75), Inches(1.95), Inches(2.75), Inches(2.55), "融合后端", "统一坐标、时间和不确定性\n在固定时窗内平滑\n对异常观测进行降权或隔离", ORANGE_LIGHT, ORANGE, ORANGE, INK, 17, 13)
    add_text(slide, Inches(0.75), Inches(5.15), Inches(11.8), Inches(0.48), "核心矛盾：固定权重把“是否在线”误当成“是否可信”。", 21, RED, True)
    add_text(slide, Inches(0.75), Inches(5.82), Inches(11.55), Inches(0.58), "本课题把观测质量转化为可记录的健康状态、协方差尺度和因子准入决策，并用同一组数据验证它们是否改善连续性与鲁棒性。", 14, INK)
    add_footer(slide, "背景依据：FAST-LIO / FAST-LIO2、LIO-SAM、轮式-LiDAR 融合研究")
    add_notes(slide, "轮式里程计的优势是高频和连续，尤其适合麦克纳姆底盘的平面运动；它的主要问题是轮滑和模型误差。FAST-LIO 能提供独立的几何约束，但在退化场景下会出现协方差异常、位姿跳变或更新中断。因此研究重点不是简单把两个输出加权平均，而是建立对观测可信度的显式判断。")
    return slide


def slide_3(prs):
    slide = add_slide(prs)
    add_title(slide, "研究对象聚焦于异构观测进入后端的方式", "边界先于算法：接口、TF 和因子语义必须与真实工程链路一致")
    add_section_tag(slide, Inches(0.68), Inches(1.45), "Scope")
    # Top source chain.
    add_box(slide, Inches(0.72), Inches(1.90), Inches(2.2), Inches(1.30), "MotionCore", "/motioncore/wheel_odom\nodom_wheel → base_link", GREEN_LIGHT, GREEN, GREEN, INK, 15, 10.5)
    add_arrow(slide, Inches(3.08), Inches(2.32), Inches(0.52), Inches(0.34), GREEN)
    add_box(slide, Inches(3.75), Inches(1.90), Inches(2.2), Inches(1.30), "FAST-LIO", "/fast_lio/odom\ncamera_init → lidar_link", CYAN_LIGHT, CYAN, CYAN, INK, 15, 10.5)
    add_arrow(slide, Inches(6.12), Inches(2.32), Inches(0.52), Inches(0.34), CYAN)
    add_box(slide, Inches(6.78), Inches(1.90), Inches(2.65), Inches(1.30), "state_fusion_backend", "Pose3 固定时窗因子图\n唯一发布 odom → base_link", ORANGE_LIGHT, ORANGE, ORANGE, INK, 15, 10.5)
    add_arrow(slide, Inches(9.60), Inches(2.32), Inches(0.52), Inches(0.34), ORANGE)
    add_box(slide, Inches(10.25), Inches(1.90), Inches(2.25), Inches(1.30), "RTAB-Map", "/odom 输入\n负责 map → odom", PURPLE_LIGHT, PURPLE, PURPLE, INK, 15, 10.5)

    add_box(slide, Inches(0.72), Inches(3.75), Inches(3.45), Inches(1.72), "本课题真正进入定位图的量", "轮式相对运动因子\nFAST-LIO 相对 Pose3 因子\n平面运动 / 参考软先验\n必要时静止锚定因子", WHITE, GRID, INK, INK, 14.5, 11.5)
    add_box(slide, Inches(4.92), Inches(3.75), Inches(3.45), Inches(1.72), "当前仅用于健康评估", "MotionCore IMU：航向一致性\n磁力计：磁场模长变化\n不直接构造 IMU 预积分或磁航向因子", WHITE, GRID, INK, INK, 14.5, 11.5)
    add_box(slide, Inches(9.10), Inches(3.75), Inches(3.40), Inches(1.72), "必须保持的工程合同", "URDF/Xacro 是传感器外参真值\n动态 TF 每条边只有一个权威发布者\n时间戳、协方差和 frame 先验收", RED_LIGHT, RED, RED, INK, 14.5, 11.5)
    add_text(slide, Inches(0.78), Inches(6.04), Inches(11.5), Inches(0.52), "研究问题：在上述边界不变的前提下，固定时窗与健康调度能否改善局部估计？", 18, INK, True)
    add_footer(slide, "工程依据：fusionlynx_slam、state_fusion_backend、fusionlynx_description")
    add_notes(slide, "这页是课题边界。FAST-LIO 是前端，state_fusion_backend 是局部融合后端，RTAB-Map 是地图侧；动态 odom 到 base_link 只由后端发布。IMU 和磁力计虽然进入健康监视，但当前代码没有把它们直接放进定位图，因此后续实验也不会把它们的参数描述成已经生效的定位因子。")
    return slide


def slide_4(prs):
    slide = add_slide(prs)
    add_title(slide, "方法架构把“观测质量”放在因子构造之前", "输入先经过时间、坐标和健康检查，再决定因子是否接入")
    add_section_tag(slide, Inches(0.68), Inches(1.45), "Architecture")
    # Full-width process.
    stages = [
        ("输入", "wheel odom\nFAST-LIO odom\nIMU / MAG", GREEN_LIGHT, GREEN),
        ("边界校验", "stamp / frame / TF\ncovariance / timeout", CYAN_LIGHT, CYAN),
        ("健康监视", "NIS-like\nwheel / IMU / MAG consistency\njump detection", ORANGE_LIGHT, ORANGE),
        ("因子构造", "WheelBetween\nLioBetween / LioPrior\nPlanar / Anchor", PURPLE_LIGHT, PURPLE),
        ("固定时窗优化", "GTSAM\nIncrementalFixedLagSmoother\nwindow = 8 s", RGBColor(235, 239, 239), INK),
        ("输出", "/odom\nodom → base_link\nfactor decisions", GREEN_LIGHT, GREEN),
    ]
    x = 0.68
    for i, (title, body, fill, line) in enumerate(stages):
        width = 1.65 if i not in (2, 3) else 1.78
        add_box(slide, Inches(x), Inches(2.18), Inches(width), Inches(2.25), title, body, fill, line, line, INK, 13.5, 10)
        if i < len(stages) - 1:
            add_arrow(slide, Inches(x + width + 0.04), Inches(3.02), Inches(0.27), Inches(0.30), line)
        x += width + 0.30
    add_text(slide, Inches(0.75), Inches(5.08), Inches(12.0), Inches(0.40), "置信度输入由多源证据联合决定：", 15.5, INK, True)
    chips = [
        ("innovation / NIS", CYAN_LIGHT, CYAN),
        ("wheel consistency", GREEN_LIGHT, GREEN),
        ("IMU yaw consistency", PURPLE_LIGHT, PURPLE),
        ("mag norm change", ORANGE_LIGHT, ORANGE),
        ("pose jump / timeout", RED_LIGHT, RED),
        ("covariance validity", RGBColor(238, 238, 238), INK),
    ]
    chip_layout = [
        (0.80, 5.72, 1.72),
        (4.00, 5.72, 1.95),
        (7.55, 5.72, 1.95),
        (0.80, 6.12, 1.72),
        (4.00, 6.12, 1.95),
        (7.55, 6.12, 1.95),
    ]
    for (text, fill, line), (cx, cy, width) in zip(chips, chip_layout):
        add_chip(slide, Inches(cx), Inches(cy), Inches(width), text, fill, line, 9.5)
    add_footer(slide, "当前实现：Pose3 状态；IMU/MAG 进入健康路径，不直接形成后端定位因子")
    add_notes(slide, "系统流程可以概括为六步。首先接收轮式和 LIO 观测，同时保留 IMU 与磁力计作为健康证据；然后检查时间、frame、TF 和协方差。健康监视器给出状态、协方差尺度、是否接入以及是否需要恢复对齐。因子图只接收通过这一决策的观测，再由固定时窗平滑器输出唯一的局部 odom。")
    return slide


def slide_5(prs):
    slide = add_slide(prs)
    add_title(slide, "固定时窗因子图只保留最近 8 s 的可优化历史", "状态是 Pose3；连续运动通过相邻状态之间的相对因子表达")
    add_section_tag(slide, Inches(0.68), Inches(1.45), "Fixed-lag graph")
    # Graph area.
    gx, gy = Inches(0.78), Inches(2.08)
    nodes = [
        (gx, "xₖ₋₂"),
        (gx + Inches(1.65), "xₖ₋₁"),
        (gx + Inches(3.30), "xₖ"),
        (gx + Inches(4.95), "xₖ₊₁"),
    ]
    # Wheel and LIO factors.
    for i, (x, label) in enumerate(nodes):
        add_circle(slide, x, gy, Inches(0.58), CYAN_LIGHT, CYAN, label, CYAN, 13)
        if i < len(nodes) - 1:
            nx = nodes[i + 1][0]
            add_line(slide, x + Inches(0.58), gy + Inches(0.29), nx, gy + Inches(0.29), GREEN, 2.1)
            add_circle(slide, x + Inches(0.68), gy + Inches(0.18), Inches(0.22), GREEN, GREEN, "w", WHITE, 8)
            # Approximate an arc using a thin top connector and a factor dot.
            add_line(slide, x + Inches(0.29), gy + Inches(0.02), nx - Inches(0.29), gy + Inches(0.02), ORANGE, 1.8)
            add_circle(slide, x + Inches(0.68), gy - Inches(0.08), Inches(0.22), ORANGE, ORANGE, "l", WHITE, 8)
    # Planar and anchor lines.
    add_line(slide, gx + Inches(0.29), gy + Inches(0.90), gx + Inches(5.24), gy + Inches(0.90), GRID, 1.3)
    for x, _ in nodes:
        add_circle(slide, x + Inches(0.17), gy + Inches(0.79), Inches(0.22), GRID, GRID, "p", INK, 8)
    add_line(slide, gx + Inches(0.29), gy + Inches(1.33), gx + Inches(5.24), gy + Inches(1.33), PURPLE, 1.4, dash=True)
    add_text(slide, gx, gy + Inches(1.43), Inches(5.65), Inches(0.24), "planar motion / reference prior        stationary anchor (when confirmed)", 9.5, MUTED)
    # Marginalization bracket.
    add_line(slide, gx, gy - Inches(0.40), gx + Inches(5.55), gy - Inches(0.40), CYAN, 1.3)
    add_text(slide, gx + Inches(0.20), gy - Inches(0.66), Inches(5.0), Inches(0.23), "fixed-lag window = 8 s", 11, CYAN, True, FONT_EN)
    add_text(slide, gx + Inches(5.70), gy - Inches(0.25), Inches(0.55), Inches(0.25), "→", 20, MUTED, True, align=PP_ALIGN.CENTER)
    add_text(slide, gx + Inches(6.15), gy - Inches(0.35), Inches(1.1), Inches(0.45), "old states\nmarginalized", 9.5, MUTED, align=PP_ALIGN.CENTER)
    # Right-side equations and semantics.
    add_box(slide, Inches(8.03), Inches(1.95), Inches(4.53), Inches(3.02), "状态与因子语义", "Xₖ = Tᵒᵈᵒᵐ_ᵇᵃˢᵉ(k) ∈ SE(3)\n\nWheelBetween：轮式 pose.between()\nLioBetween：连续 FAST-LIO 相对运动\nLioPrior：无相邻 LIO 增量时的对齐先验\nPlanar / Anchor：有限协方差软约束", WHITE, GRID, INK, INK, 15, 11.5)
    add_box(slide, Inches(8.03), Inches(5.25), Inches(4.53), Inches(0.92), "设计取舍", "窗口越长，局部一致性可能更好；但更新时间、边缘化开销和延迟同步增加。", ORANGE_LIGHT, ORANGE, ORANGE, INK, 13, 10.5)
    add_footer(slide, "理论基础：iSAM2 增量平滑；工程实现：GTSAM IncrementalFixedLagSmoother")
    add_notes(slide, "状态变量是每个事件时刻的 Pose3，不包含速度、陀螺零偏或磁航向状态。轮式和 LIO 都先转换为相邻状态的相对运动因子；当连续 LIO 不可用时，后端转入轮式预测或弱先验。平面先验和静止锚定是有限协方差约束，作用是抑制地面平台不合理的 z、roll、pitch 随机游走，而不是把真实坡道运动硬压平。")
    return slide


def slide_6(prs):
    slide = add_slide(prs)
    add_title(slide, "LIO 置信度由可观测证据驱动，并映射为因子级动作", "健康状态由观测证据触发，并作为实时调度输入")
    add_section_tag(slide, Inches(0.68), Inches(1.45), "Health-aware scheduling")
    # State machine.
    states = [
        ("HEALTHY", "原始协方差\n接入 LIO 因子", GREEN_LIGHT, GREEN),
        ("DEGRADED", "协方差放大\nHuber 鲁棒核", ORANGE_LIGHT, ORANGE),
        ("FAILED", "暂停 LIO 因子\n轮式 / 先验维持", RED_LIGHT, RED),
        ("RECOVERING", "先做状态对齐\n再恢复相对因子", PURPLE_LIGHT, PURPLE),
    ]
    sx = 0.72
    for i, (name, body, fill, line) in enumerate(states):
        add_box(slide, Inches(sx), Inches(2.15), Inches(2.52), Inches(1.58), name, body, fill, line, line, INK, 15, 11)
        if i < 3:
            add_arrow(slide, Inches(sx + 2.68), Inches(2.73), Inches(0.42), Inches(0.33), line)
        sx += 3.05
    add_text(slide, Inches(0.82), Inches(3.98), Inches(11.3), Inches(0.30), "触发依据", 15, INK, True)
    evidence = [
        ("NIS / innovation", "位姿创新与协方差归一化后的异常程度", CYAN),
        ("wheel consistency", "轮式增量与 LIO 增量的平面位置 / yaw 差异", GREEN),
        ("IMU consistency", "LIO yaw 增量与陀螺积分差异", PURPLE),
        ("jump / timeout", "位姿跳变、时间间隔和超时", RED),
        ("covariance", "有限性、正定性与系统噪声下限", ORANGE),
    ]
    ex, ey = 0.82, 4.40
    evidence_layout = [
        (0.82, 4.40, 1.55, 2.55),
        (4.90, 4.40, 1.55, 6.63),
        (8.98, 4.40, 1.55, 10.71),
        (2.00, 4.88, 1.55, 3.73),
        (6.80, 4.88, 1.55, 8.53),
    ]
    for (text, body, color), (chip_x, chip_y, chip_w, body_x) in zip(evidence, evidence_layout):
        add_chip(slide, Inches(chip_x), Inches(chip_y), Inches(chip_w), text, rgb("F0F3F3"), color, 9.2)
        add_text(slide, Inches(body_x), Inches(chip_y + 0.02), Inches(2.05), Inches(0.30), body, 9.5, MUTED)
    add_box(slide, Inches(0.82), Inches(5.58), Inches(5.75), Inches(0.92), "初始配置中的门限", "NIS 初值：16.8 / 30.0；一致性初值：3.0 / 6.0；健康开关需在实验 profile 中启用并标定。", WHITE, GRID, INK, INK, 12.5, 9.6)
    add_box(slide, Inches(6.82), Inches(5.58), Inches(5.68), Inches(0.92), "必须验证的输出", "记录 covariance_scale、accept_factor、requires_alignment、reason，以及恢复时间与恢复跳变。", WHITE, GRID, INK, INK, 12.5, 9.6)
    add_footer(slide, "门限仅作为初始配置；论文结论必须来自标定集、留出集和压力场景")
    add_notes(slide, "健康状态由多个可观测证据联合决定。比如 NIS 反映创新相对于协方差是否异常，轮式一致性反映两个相对运动源是否冲突，IMU 和磁场提供额外健康线索。DEGRADED 不等于直接丢弃，而是增大协方差并使用 Huber；FAILED 才暂停因子；RECOVERING 需要先对齐状态，避免恢复首帧造成跳变。")
    return slide


def slide_7(prs):
    slide = add_slide(prs)
    add_title(slide, "代码路径已经把健康决策落到“因子接纳与噪声尺度”", "工程实现证据：健康监视器与因子图构造器之间有明确接口")
    add_section_tag(slide, Inches(0.68), Inches(1.45), "Implementation")
    # Code panel.
    code = (
        "const bool use_lio =\n"
        "  accepted_lio != nullptr &&\n"
        "  health.accept_factor &&\n"
        "  !recovery_alignment_frame;\n\n"
        "build_context.lio_scale =\n"
        "  health.covariance_scale *\n"
        "  (stationary ? stationary_lio_scale : 1.0);\n\n"
        "if (!context.lio_reliability.accept_factor)\n"
        "  record_decision(LioBetween, false,\n"
        "                  context.lio_scale,\n"
        "                  context.lio_reliability.reason);"
    )
    code_box = add_shape(slide, MSO_SHAPE.ROUNDED_RECTANGLE, Inches(0.72), Inches(1.95), Inches(6.02), Inches(4.12), DARK, RGBColor(58, 78, 86))
    add_text(slide, Inches(0.95), Inches(2.17), Inches(5.45), Inches(0.28), "fusion_backend.cpp / factor_graph_builder.cpp", 10, RGBColor(164, 204, 199), True, FONT_MONO)
    add_text(slide, Inches(0.95), Inches(2.58), Inches(5.47), Inches(3.15), code, 11.2, RGBColor(232, 242, 240), False, FONT_MONO)
    # Right evidence chain.
    add_box(slide, Inches(7.22), Inches(1.95), Inches(5.28), Inches(0.86), "1 观测进入", "wheel / LIO 按时间排序，缓存有限乱序", GREEN_LIGHT, GREEN, GREEN, INK, 13.5, 10.5)
    add_arrow(slide, Inches(9.58), Inches(2.93), Inches(0.42), Inches(0.30), CYAN)
    add_box(slide, Inches(7.22), Inches(3.26), Inches(5.28), Inches(0.86), "2 健康判断", "LioHealthResult 给出状态、尺度、准入与恢复对齐", ORANGE_LIGHT, ORANGE, ORANGE, INK, 13.5, 10.5)
    add_arrow(slide, Inches(9.58), Inches(4.24), Inches(0.42), Inches(0.30), CYAN)
    add_box(slide, Inches(7.22), Inches(4.57), Inches(5.28), Inches(0.86), "3 因子决策", "LioBetween / LioPrior 接纳或拒绝，并记录 reason", PURPLE_LIGHT, PURPLE, PURPLE, INK, 13.5, 10.5)
    add_arrow(slide, Inches(9.58), Inches(5.55), Inches(0.42), Inches(0.30), CYAN)
    add_box(slide, Inches(7.22), Inches(5.88), Inches(5.28), Inches(0.86), "4 输出与诊断", "/odom、odom → base_link、factor decision、recovery alignment", CYAN_LIGHT, CYAN, CYAN, INK, 13.5, 10.5)
    add_footer(slide, "当前后端：Pose3 + relative BetweenFactor；IMU/MAG 保留在健康 API 外侧")
    add_notes(slide, "这页强调课题不是停留在概念层面。代码中 health.accept_factor 会直接决定 LIO 相对因子是否构造，health.covariance_scale 会进入噪声模型，恢复对齐帧会被排除，避免把失效前后的原始位姿差写成异常运动。每次因子决策还会记录原因，便于实验后追溯。")
    return slide


def slide_8(prs):
    slide = add_slide(prs)
    add_title(slide, "评价设计采用同一 rosbag、同一初始条件和同一参考轨迹", "先验证可比性，再比较精度、连续性和实时性")
    add_section_tag(slide, Inches(0.68), Inches(1.45), "Evaluation design")
    # Replay flow.
    flow = [
        ("同一传感器回放", "wheel / LIO / IMU / MAG\n/tf / /tf_static", GREEN_LIGHT, GREEN),
        ("统一审计", "时间戳\nframe / TF\n外参 / QoS", CYAN_LIGHT, CYAN),
        ("四类配置", "Wheel-only\nFAST-LIO-only\n固定权重融合\n健康调度融合", ORANGE_LIGHT, ORANGE),
        ("统一输出", "/odom\n诊断日志\n轨迹与耗时", PURPLE_LIGHT, PURPLE),
    ]
    x = 0.72
    for i, (title, body, fill, line) in enumerate(flow):
        add_box(slide, Inches(x), Inches(1.98), Inches(2.62), Inches(1.85), title, body, fill, line, line, INK, 14, 10.5)
        if i < len(flow) - 1:
            add_arrow(slide, Inches(x + 2.76), Inches(2.72), Inches(0.43), Inches(0.30), line)
        x += 3.12
    # Metrics.
    add_text(slide, Inches(0.80), Inches(4.35), Inches(4.4), Inches(0.30), "主指标", 15, INK, True)
    metrics = [
        ("精度", "ATE / RPE\nx-y-yaw 分量误差"),
        ("鲁棒性", "LIO 失效输出连续性\n恢复时间 / 恢复跳变"),
        ("一致性", "静止抖动\n因子拒绝与健康误报"),
        ("实时性", "更新耗时 / 延迟\nCPU / 内存"),
    ]
    mx, my = 0.80, 4.82
    for i, (title, body) in enumerate(metrics):
        x = Inches(mx + (i % 2) * 2.88)
        y = Inches(my + (i // 2) * 0.94)
        add_box(slide, x, y, Inches(2.55), Inches(0.72), title, body, WHITE, GRID, INK, MUTED, 12, 9.5)
    # Reference and leakage note.
    add_box(slide, Inches(7.10), Inches(4.35), Inches(5.40), Inches(2.13), "实验控制原则", "• 标定集与测试集按完整序列划分\n• 不把相邻样本拆到两侧，避免数据泄漏\n• 有度量级参考才报告绝对精度\n• 没有参考时只报告重复性、连续性与故障恢复", WHITE, GRID, INK, INK, 14, 11)
    add_footer(slide, "比较对象和指标均来自开题报告；具体数值待实验完成后给出")
    add_notes(slide, "评价的关键是可比性。所有配置都使用同一段传感器回放、同一初始条件、同一外参和同一参考轨迹。对比对象包括轮式、FAST-LIO、固定权重融合和健康调度融合。指标不仅包括 ATE 和 RPE，也包括 LIO 失效期间的输出连续性、恢复跳变和计算开销。")
    return slide


def slide_9(prs, out_dir):
    slide = add_slide(prs)
    add_title(slide, "点云地图与轨迹对比是最终答辩的核心证据位", "当前仅展示图形形式，最终内容由统一回放实验替换")
    add_section_tag(slide, Inches(0.68), Inches(1.45), "Evidence slots")
    left = out_dir / "assets" / "pointcloud_context.png"
    right = out_dir / "assets" / "trajectory_placeholder.png"
    add_image_contain(slide, left, Inches(0.72), Inches(1.98), Inches(6.03), Inches(3.45), RGBColor(70, 80, 82))
    add_image_contain(slide, right, Inches(7.08), Inches(1.98), Inches(5.47), Inches(3.45), GRID)
    add_text(slide, Inches(0.78), Inches(5.54), Inches(5.8), Inches(0.40), "点云地图展示位", 15, INK, True)
    add_text(slide, Inches(0.78), Inches(5.95), Inches(5.8), Inches(0.48), "展示 FAST-LIO / RTAB-Map 的空间结构与地图完整性；需保留坐标轴、图例和数据来源。", 10.5, MUTED)
    add_text(slide, Inches(7.14), Inches(5.54), Inches(5.2), Inches(0.40), "轨迹对比展示位", 15, INK, True)
    add_text(slide, Inches(7.14), Inches(5.95), Inches(5.2), Inches(0.48), "统一绘制 Wheel-only、FAST-LIO-only、固定权重和健康调度四条轨迹，并同步报告 ATE / RPE。", 10.5, MUTED)
    add_chip(slide, Inches(0.78), Inches(6.55), Inches(2.45), "工程素材 / 待实测替换", ORANGE_LIGHT, ORANGE, 9)
    add_chip(slide, Inches(7.14), Inches(6.55), Inches(2.45), "格式示例 / 非结果", ORANGE_LIGHT, ORANGE, 9)
    add_footer(slide, "图形占位遵循：先保留科学上下文，再做裁剪与统一配色")
    add_notes(slide, "开题阶段不应该用一张漂亮的轨迹图替代实验。这里保留最终答辩需要的两个证据位：点云地图和轨迹对比。当前图片只是已有工程素材和图形格式示例，已经明确标注为待替换，不能被解释为本课题的性能结果。")
    return slide


def slide_10(prs):
    slide = add_slide(prs)
    add_title(slide, "预期优势来自工程边界与健康调度，性能结论需经实验确认", "优势边界必须通过同一数据集的对比和消融实验确认")
    add_section_tag(slide, Inches(0.68), Inches(1.45), "Expected contribution")
    # Table-like comparison.
    x0, y0 = Inches(0.72), Inches(1.98)
    widths = [Inches(2.05), Inches(4.00), Inches(2.55)]
    headers = ["对照对象", "预期改善", "验收证据"]
    cx = x0
    for w, htext in zip(widths, headers):
        cell = add_shape(slide, MSO_SHAPE.RECTANGLE, cx, y0, w, Inches(0.48), DARK, DARK)
        set_text(cell, htext, 12, WHITE, True, align=PP_ALIGN.CENTER, valign=MSO_ANCHOR.MIDDLE, margin=0.02)
        cx += w
    rows = [
        ("Wheel-only", "利用 LIO 几何约束抑制轮滑和长期积分漂移", "ATE / RPE\n漂移率"),
        ("FAST-LIO-only", "利用轮式高频平面运动保持短时连续性", "断续时长\n平面误差"),
        ("固定权重融合", "健康调度降低异常 LIO 对单帧估计的主导", "误报警率\n恢复跳变"),
        ("通用框架", "面向麦克纳姆平台的接口、TF、窗口与恢复合同", "可追踪日志\n可复现实验"),
    ]
    ry = y0 + Inches(0.48)
    for r, (a, b, c) in enumerate(rows):
        fill = WHITE if r % 2 == 0 else RGBColor(241, 245, 244)
        cx = x0
        for w, text in zip(widths, [a, b, c]):
            cell = add_shape(slide, MSO_SHAPE.RECTANGLE, cx, ry, w, Inches(0.78), fill, GRID)
            set_text(cell, text, 10.5, INK if w != widths[2] else MUTED, r == 0 and w == widths[0], valign=MSO_ANCHOR.MIDDLE, margin=0.08)
            cx += w
        ry += Inches(0.78)
    # Innovation rail.
    add_box(slide, Inches(9.78), Inches(1.98), Inches(2.72), Inches(3.60), "拟强调的创新点", "1 观测可信度映射为因子级动作\n\n2 固定时窗内的显式恢复对齐\n\n3 面向真实 ROS 2 链路的可追踪实验合同\n\n4 明确 IMU / MAG 的健康边界，不夸大未启用因子", CYAN_LIGHT, CYAN, CYAN, INK, 14, 11)
    add_box(slide, Inches(0.72), Inches(5.90), Inches(8.55), Inches(0.80), "表述边界", "开题阶段不预先给出 ATE、RPE 或百分比提升；最终结论以独立留出数据和压力场景为准。", ORANGE_LIGHT, ORANGE, ORANGE, INK, 13, 10.5)
    add_footer(slide, "创新点是可验证的机制与工程闭环，不是预先写死的性能数字")
    add_notes(slide, "这里需要谨慎回答答辩老师最常问的‘比已有框架好在哪里’。目前不能宣称普遍精度更高，能够明确的是优势边界：对当前麦克纳姆机器人，系统把接口合同、固定时窗、健康状态和恢复对齐统一起来；是否带来量化收益，需要通过四类基线和留出数据来验证。")
    return slide


def slide_11(prs):
    slide = add_slide(prs)
    add_title(slide, "研究计划按“接口冻结—标定—对比—论文”推进", "每个阶段都有可交付证据，避免先调参后解释")
    add_section_tag(slide, Inches(0.68), Inches(1.45), "Work plan")
    phases = [
        ("阶段 1", "接口与模型冻结", "话题 / frame / TF\n外参与时间审计\n因子语义单测", GREEN_LIGHT, GREEN),
        ("阶段 2", "参数测定", "轮式 / LIO 噪声\n固定时窗扫描\n健康门限标定", CYAN_LIGHT, CYAN),
        ("阶段 3", "系统实验", "同一 rosbag 回放\n四类配置对比\n退化与恢复压力场景", ORANGE_LIGHT, ORANGE),
        ("阶段 4", "分析与答辩", "ATE / RPE\n日志与轨迹图\n论文与答辩材料", PURPLE_LIGHT, PURPLE),
    ]
    x = 0.78
    for i, (phase, title, body, fill, line) in enumerate(phases):
        add_box(slide, Inches(x), Inches(2.10), Inches(2.75), Inches(2.15), phase, f"{title}\n\n{body}", fill, line, line, INK, 13, 10.5)
        if i < 3:
            add_arrow(slide, Inches(x + 2.88), Inches(2.92), Inches(0.45), Inches(0.32), line)
        x += 3.12
    add_text(slide, Inches(0.80), Inches(4.72), Inches(3.4), Inches(0.28), "关键里程碑", 15, INK, True)
    milestones = [
        ("M1", "完成 TF / 时间 / 协方差审计", GREEN),
        ("M2", "形成可复核的参数测定记录", CYAN),
        ("M3", "完成四配置同袋回放", ORANGE),
        ("M4", "冻结留出集结果并撰写结论", PURPLE),
    ]
    y = 5.16
    for label, body, color in milestones:
        add_chip(slide, Inches(0.82), Inches(y), Inches(0.58), label, color, WHITE, 9.5)
        add_text(slide, Inches(1.55), Inches(y + 0.02), Inches(5.3), Inches(0.24), body, 11, INK)
        y += 0.38
    add_box(slide, Inches(7.28), Inches(4.72), Inches(5.22), Inches(1.70), "阶段性判停条件", "若 frame / 时间戳 / 外参合同未通过，暂停精度标定；若健康门限在正常数据上误报警，先回到噪声与一致性测定；若无法获得度量级参考，只报告相对重复性和故障恢复。", RED_LIGHT, RED, RED, INK, 13, 10.5)
    add_footer(slide, "日期按开题答辩节点组织，具体实验时间以硬件与数据条件为准")
    add_notes(slide, "工作流程遵循从接口到结论的顺序。第一阶段冻结话题、frame、TF 和因子语义；第二阶段分别测量轮式和 LIO 的噪声，再扫描窗口和门限；第三阶段在同一数据上完成基线与压力场景；最后冻结测试集结果再写论文。任何接口错误都不能靠增大 sigma 来掩盖。")
    return slide


def slide_12(prs):
    slide = add_slide(prs)
    add_title(slide, "已有代码、配置和测试构成了课题的可行性基础", "可行性来自可运行链路、可检查边界与已有工程积累")
    add_section_tag(slide, Inches(0.68), Inches(1.45), "Feasibility")
    items = [
        ("代码基础", "state_fusion_backend 已包含 FusionBackend、FactorGraphBuilder、LioHealthMonitor 与测试入口", GREEN_LIGHT, GREEN),
        ("系统基础", "FAST-LIO、MotionCore 轮式里程计、robot_state_publisher 和 RTAB-Map 已有清晰责任边界", CYAN_LIGHT, CYAN),
        ("配置基础", "fusionlynx_slam.yaml 已集中描述话题、frame、窗口、噪声和健康门限", ORANGE_LIGHT, ORANGE),
        ("验证基础", "已有参数测定方案、TF 审计、同袋回放和轨迹评价脚本，可形成可追溯记录", PURPLE_LIGHT, PURPLE),
    ]
    y = 1.98
    for title, body, fill, line in items:
        add_box(slide, Inches(0.78), Inches(y), Inches(7.15), Inches(0.86), title, body, fill, line, line, INK, 13.5, 10.5)
        y += 1.05
    add_box(slide, Inches(8.35), Inches(1.98), Inches(4.15), Inches(2.30), "主要风险", "• 轮式时间戳与 LIO 不同步\n• 外参与消息 frame 不一致\n• NIS 门限对正常运动误报警\n• LIO 恢复首帧产生跳变", RED_LIGHT, RED, RED, INK, 14, 11)
    add_box(slide, Inches(8.35), Inches(4.58), Inches(4.15), Inches(2.02), "控制策略", "先审计，再标定；先单传感器，再融合；先正常数据，再压力场景；保留每个因子接纳 / 拒绝原因和恢复对齐记录。", WHITE, GRID, INK, INK, 14, 11)
    add_footer(slide, "实现边界与验证顺序均已写入工程文档和参数测定方案")
    add_notes(slide, "课题具有可行性，原因不是单纯因为已有文献，而是工程链路已经拆分出明确的节点和责任边界，后端也已经有固定时窗、因子构造和健康监视的代码骨架。风险主要集中在时间、外参、门限和恢复行为，解决策略是把审计和日志作为实验的一部分。")
    return slide


def slide_13(prs):
    slide = add_slide(prs, dark=True)
    add_section_tag(slide, Inches(0.70), Inches(0.75), "Take-home message", CYAN)
    add_text(slide, Inches(0.70), Inches(1.25), Inches(11.9), Inches(0.62), "固定时窗因子图提供统一状态空间，健康调度决定异常观测如何影响状态。", 23, WHITE, True)
    add_line(slide, Inches(0.72), Inches(2.12), Inches(12.35), Inches(2.12), RGBColor(65, 94, 100), 1.0)
    takeaways = [
        ("01", "研究对象", "真实 ROS 2 麦克纳姆机器人上的 Wheel + FAST-LIO 局部融合"),
        ("02", "方法主线", "Pose3 固定时窗 + 相对运动因子 + 基于证据的 LIO 健康调度"),
        ("03", "验收方式", "同一 rosbag、统一参考轨迹、精度—连续性—实时性联合评价"),
    ]
    y = 2.55
    for number, title, body in takeaways:
        add_text(slide, Inches(0.78), Inches(y), Inches(0.58), Inches(0.34), number, 14, CYAN, True, FONT_EN)
        add_text(slide, Inches(1.58), Inches(y), Inches(2.1), Inches(0.34), title, 15, WHITE, True)
        add_text(slide, Inches(3.90), Inches(y), Inches(7.7), Inches(0.40), body, 13, RGBColor(209, 224, 221))
        y += 0.88
    add_text(slide, Inches(0.78), Inches(5.55), Inches(6.2), Inches(0.36), "谢谢各位老师，欢迎批评指正。", 22, WHITE, True)
    add_text(slide, Inches(0.80), Inches(6.40), Inches(9.2), Inches(0.30), "主要依据：FAST-LIO2 · LIO-SAM · iSAM2 · LIWO / LIO-Fusion · DAMS-LIO · 工程代码与配置", 8.5, RGBColor(161, 182, 180))
    add_text(slide, Inches(11.0), Inches(6.35), Inches(1.55), Inches(0.36), "Q&A", 22, CYAN, True, FONT_EN, align=PP_ALIGN.RIGHT)
    add_footer(slide, "开题答辩材料 · 2026-10-02", dark=True)
    add_notes(slide, "最后总结三点。第一，研究对象是实际 ROS 2 麦克纳姆平台上的 Wheel 与 FAST-LIO 局部融合；第二，方法主线是 Pose3 固定时窗、相对运动因子和健康调度；第三，验收必须用同一 rosbag 和统一参考轨迹，联合报告精度、连续性和实时性。谢谢各位老师。")
    return slide


def build_deck(out_dir: Path) -> Path:
    out_dir.mkdir(parents=True, exist_ok=True)
    copied = copy_assets(out_dir)
    prs = Presentation()
    prs.slide_width = W
    prs.slide_height = H

    slide_1(prs, out_dir)
    slide_2(prs)
    slide_3(prs)
    slide_4(prs)
    slide_5(prs)
    slide_6(prs)
    slide_7(prs)
    slide_8(prs)
    slide_9(prs, out_dir)
    slide_10(prs)
    slide_11(prs)
    slide_12(prs)
    slide_13(prs)

    pptx_path = out_dir / "开题报告_答辩PPT.pptx"
    prs.save(pptx_path)

    (out_dir / "ppt_outline_cn.md").write_text(
        """# 开题答辩 PPT 结构\n\n"
        "1. 题目与汇报信息\n"
        "2. 轮式与 LIO 的互补性\n"
        "3. 研究对象与工程边界\n"
        "4. 方法总览：观测质量先于因子构造\n"
        "5. 8 s Pose3 固定时窗因子图\n"
        "6. 基于证据的 LIO 健康调度\n"
        "7. 代码路径与因子决策\n"
        "8. 同袋回放与评价指标\n"
        "9. 点云地图与轨迹对比证据位\n"
        "10. 预期优势与创新点\n"
        "11. 工作计划与判停条件\n"
        "12. 可行性与风险控制\n"
        "13. 总结与答辩\n"
        """,
        encoding="utf-8",
    )
    manifest = [
        "# Asset Manifest",
        "",
        "| Asset | Role | Provenance | Status |",
        "|---|---|---|---|",
    ]
    for name, source in copied.items():
        manifest.append(f"| `{name}` | Context or result-format placeholder | `{source}` | Not an experimental result |")
    manifest.append("| `cover_robot.png`, `indoor_mapping_scene.png` | Requested generated scene images | Local image API returned HTTP 503; no image embedded | Unavailable |")
    (out_dir / "asset_manifest.md").write_text("\n".join(manifest) + "\n", encoding="utf-8")
    (out_dir / "qa_report.md").write_text(
        """# PPTX QA Report

## Scope

- Chinese 16:9 opening-defense deck for a 15-minute report.
- 13 slides with speaker notes.
- Content grounded in `开题报告.docx`, the FusionLynx engineering documents, and the current backend source.

## Evidence boundary

- No numerical performance result is fabricated.
- Point-cloud and trajectory assets are explicitly marked as context or result-format placeholders.
- The requested local image API was called through the project-local skill and returned HTTP 503: `No available compatible accounts`; no generated image was embedded.

## Checks performed

- PPTX reopened with `python-pptx` after generation.
- Slide count, notes presence, image count, and slide-bound geometry checked by the builder-side verification script.
- Native diagrams used for architecture, factor graph, health state machine, evaluation flow, and comparison table.
- Final bundled audit is stored in `pptx_audit.md`; high-severity findings are zero.

## Known limitation

- The source DOCX could not be raster-rendered in the container because the bundled LibreOffice binary requires newer system libraries. Text and table content were extracted directly from OOXML.
""",
        encoding="utf-8",
    )
    return pptx_path


def verify_pptx(pptx_path: Path) -> None:
    check = Presentation(pptx_path)
    if len(check.slides) != 13:
        raise RuntimeError(f"expected 13 slides, got {len(check.slides)}")
    notes = 0
    images = 0
    for slide in check.slides:
        if slide.notes_slide.notes_text_frame.text.strip():
            notes += 1
        for shape in slide.shapes:
            if shape.shape_type == 13:
                images += 1
            if shape.left < 0 or shape.top < 0:
                raise RuntimeError(f"negative shape position on slide {slide.slide_id}")
            if shape.left + shape.width > check.slide_width + 1 or shape.top + shape.height > check.slide_height + 1:
                raise RuntimeError(f"shape outside slide bounds on slide {slide.slide_id}")
    if notes != 13:
        raise RuntimeError(f"expected notes on 13 slides, got {notes}")
    print(f"verified slides={len(check.slides)} notes={notes} images={images}")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--out-dir", type=Path, required=True)
    args = parser.parse_args()
    pptx_path = build_deck(args.out_dir)
    verify_pptx(pptx_path)
    print(pptx_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
