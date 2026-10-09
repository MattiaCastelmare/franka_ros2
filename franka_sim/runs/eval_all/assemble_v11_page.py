"""Inject v11 data + text into the template -> OUT/index.html.   assemble_v11_page.py DATA.json OUT_DIR"""
import json, sys, os
D = json.load(open(sys.argv[1])); out = sys.argv[2]; os.makedirs(out, exist_ok=True)
H = os.path.dirname(os.path.abspath(__file__))
C = D['conf']; W = D['win']; B = 'sac_v10c:2500000'
w, b = C[W], C[B]
g = D['grid']['sac_v11_ft_prec']['3750000']
n = w['dynamic']['n']
def pct(x, n): return f'{100 * x / n:.0f}%'
TXT = dict(
  h1='Batch v11: il bonus di precisione batte v10c, soprattutto con l’ostacolo fermo',
  lead=('Otto training in parallelo (30 set, 19:00 → 1 ott, ~11 h). Il migliore, <span class="mono">ft_prec</span> a 3.75M, '
        'è v10c 2.5M allenato altri 1.25M step a learning rate 1e-4 con un bonus che cresce negli ultimi centimetri dal target. '
        f'Sui {n} seed nuovi, mai usati per sceglierlo, tiene il target più spesso di v10c con l’ostacolo fermo (+6 punti) e un po’ più spesso con quello mobile (+3.5 punti, non significativo), senza collisioni.'),
  big=[[f'{w["static"]["held"]}<em>/{n}</em>', f'fermo, tenuti (v10c {b["static"]["held"]}) · p {w["static"]["p"]:.3f}'],
       [f'{w["dynamic"]["held"]}<em>/{n}</em>', f'mobile, tenuti (v10c {b["dynamic"]["held"]}) · p {w["dynamic"]["p"]:.2f}'],
       [f'{w["dynamic"]["coll"] + w["static"]["coll"]}', f'collisioni in {2 * n} episodi'],
       [f'{g["dynamic"]["held"]} · {g["static"]["held"]}', 'su 140 seed del benchmark (v10c 112 · 92)']],
  confh=f'Su {n} seed nuovi resta un guadagno netto con l’ostacolo fermo',
  pairh=f'ft_prec 3.75M contro v10c 2.5M sugli stessi {n} seed nuovi',
  vidh='Stessi 10 episodi nello stesso istante: ft_prec 3.75M sopra, v10c 2.5M sotto',
  concl=json.load(open(H + '/v11_concl.json')),
)
html = open(H + '/v11_page_template.html').read()
html = html.replace('window.D=/*DATA*/null;', 'window.D=' + json.dumps(D, separators=(',', ':')) + ';')
html = html.replace('window.TXT=/*TXT*/null;', 'window.TXT=' + json.dumps(TXT, ensure_ascii=False) + ';')
open(out + '/index.html', 'w').write(html)
print('wrote', out + '/index.html', len(html) // 1024, 'kB')
