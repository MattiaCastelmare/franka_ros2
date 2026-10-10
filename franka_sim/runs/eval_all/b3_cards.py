"""Big, plain title/result cards for the "who wins" slide video (2026-10-06).  python3 b3_cards.py OUTDIR
1920x1084 PNGs (= two stacked 1920x542 panels). Numbers are the all-hard-scene counts (runs/videos/b3_slide/stats*)."""
import os, sys
from PIL import Image, ImageDraw, ImageFont
OUT = sys.argv[1]; os.makedirs(OUT, exist_ok=True)
W, H = 1920, 1084
F = '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'; FB = '/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf'
BG, FG, MUT = (16, 18, 17), (242, 244, 242), (150, 158, 152)
ENS, FT = (40, 190, 135), (240, 120, 60)
def font(sz, bold=False): return ImageFont.truetype(FB if bold else F, sz)
def card(name, rows):
    """rows: (text | [(text, colour), ...], size, bold, colour, gap_before)"""
    img = Image.new('RGB', (W, H), BG); d = ImageDraw.Draw(img)
    blocks = []
    for text, sz, bold, col, gap in rows:
        parts = text if isinstance(text, list) else [(text, col)]
        f = font(sz, bold); w = sum(d.textlength(t, font=f) for t, _ in parts)
        blocks.append((parts, f, w, sz, gap))
    total = sum(sz * 1.25 + gap for _, _, _, sz, gap in blocks); y = (H - total) / 2
    for parts, f, w, sz, gap in blocks:
        y += gap; x = (W - w) / 2
        for t, c in parts:
            d.text((x, y), t, font=f, fill=c); x += d.textlength(t, font=f)
        y += sz * 1.25
    img.save(os.path.join(OUT, name + '.png'))
E, Fm = ('ens_soup9', ENS), ('ft_v10c', FT)
card('intro', [
    ('Which RL policy is better, and when?', 78, True, FG, 0),
    ('Both start from the same policy (v10c), trained WITH the shield', 42, False, FG, 50),
    ([('Top:  ', MUT), E, ('  kept training WITH the shield  ·  9 networks averaged', FG)], 42, False, FG, 40),
    ([('Bottom:  ', MUT), Fm, ('  fine-tuned WITHOUT the shield  ·  1 network', FG)], 42, False, FG, 16),
    ('Shield = keeps the robot at least 15 cm from the obstacle', 38, False, MUT, 50),
    ('In every scene the obstacle blocks the direct path to the target', 38, False, MUT, 12)])
CASES = [
    ('1', 'Shield ON', 'the setup used on the real robot', '4 episodes: 2 static, 2 moving obstacle',
     [E, (' wins', FG)], ['Targets reached:  ', E, ('  218', FG), ('   vs   ', MUT), Fm, ('  126', FG), ('   (static)', MUT)],
     ['Targets reached:  ', E, ('  180', FG), ('   vs   ', MUT), Fm, ('  143', FG), ('   (moving)', MUT)], 'No collisions for either model'),
    ('2', 'Shield OFF  ·  static obstacle', 'what each policy learned by itself', '4 episodes',
     [Fm, (' wins', FG)], ['Collisions:  ', E, ('  84', FG), ('   vs   ', MUT), Fm, ('  23', FG)], None,
     'ens_soup9 hits the obstacle in the first second'),
    ('3', 'Shield OFF  ·  moving obstacle', 'what each policy learned by itself', '4 episodes',
     [E, (' wins', FG)], ['Targets reached:  ', E, ('  173', FG), ('   vs   ', MUT), Fm, ('  134', FG)],
     ['Collisions:  ', E, ('  20', FG), ('   vs   ', MUT), Fm, ('  26', FG)], 'Averaging 9 networks gives steadier, more precise motion')]
for n, title, sub, eps, win, l1, l2, note in CASES:
    card(f'case{n}', [(f'CASE {n} OF 3', 38, True, MUT, 0), (title, 92, True, FG, 24), (sub, 46, False, FG, 18), (eps, 38, False, MUT, 50)])
    fix = lambda l: [(t, MUT) if isinstance(t, str) else t for t in l]
    rows = [(f'CASE {n}  ·  RESULT', 38, True, MUT, 0), (win, 92, True, FG, 24), (fix(l1), 46, False, FG, 50)]
    if l2: rows.append((fix(l2), 46, False, FG, 16))
    rows.append((note, 38, False, MUT, 40)); rows.append(('counts over all hard scenes: 249 static, 199 moving', 30, False, MUT, 30))
    card(f'result{n}', rows)
card('summary', [
    ('Summary', 84, True, FG, 0),
    ([('Shield ON   →   ', FG), E], 54, False, FG, 60),
    ([('Shield OFF, static obstacle   →   ', FG), Fm], 54, False, FG, 26),
    ([('Shield OFF, moving obstacle   →   ', FG), E], 54, False, FG, 26),
    ('On the robot: ens_soup9 with the shield, 0 collisions', 42, False, MUT, 70)])
print('cards ->', OUT)
