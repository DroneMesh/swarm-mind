// Scene picker: a button showing the current map that opens a list of map
// cards (name, blurb, radio challenge). The choice persists in the URL hash
// as #map=<key>, so a link opens the same scene.

import { PRESETS, type PresetInfo } from '../presets';

/** The preset named by `#map=<key>` (or a numeric id), else Alpine Ridge. */
export function presetFromHash(hash = location.hash): number {
  const m = /(?:^#|&)map=([\w-]+)/.exec(hash);
  if (!m) return 0;
  const key = m[1].toLowerCase();
  const byKey = PRESETS.findIndex((p) => p.key === key);
  if (byKey >= 0) return byKey;
  const n = Number(key);
  return Number.isInteger(n) && n >= 0 && n < PRESETS.length ? n : 0;
}

export function writeHash(preset: number): void {
  const key = PRESETS[preset]?.key;
  if (!key) return;
  // replaceState: switching maps should not flood the back button.
  try {
    history.replaceState(null, '', `#map=${key}`);
  } catch {
    location.hash = `map=${key}`;
  }
}

export class ScenePicker {
  private readonly button: HTMLButtonElement;
  private readonly menu: HTMLDivElement;
  private readonly cards: HTMLButtonElement[] = [];
  private current = 0;

  constructor(
    root: HTMLElement,
    private readonly onPick: (preset: number) => void,
  ) {
    root.classList.add('picker');
    this.button = document.createElement('button');
    this.button.className = 'pick';
    this.button.setAttribute('aria-haspopup', 'listbox');
    this.button.setAttribute('aria-expanded', 'false');
    this.button.addEventListener('click', () => this.setOpen(!this.isOpen));
    this.menu = document.createElement('div');
    this.menu.className = 'menu';
    this.menu.hidden = true;
    this.menu.setAttribute('role', 'listbox');
    for (const p of PRESETS) this.menu.append(this.card(p));
    root.append(this.button, this.menu);
    document.addEventListener('pointerdown', (e) => {
      if (!this.menu.hidden && !root.contains(e.target as Node)) this.setOpen(false);
    });
    document.addEventListener('keydown', (e) => {
      if (e.key === 'Escape' && !this.menu.hidden) {
        this.setOpen(false);
        e.stopImmediatePropagation();
      }
    });
  }

  get isOpen(): boolean {
    return !this.menu.hidden;
  }

  /** Shows `preset` as current without firing `onPick`. */
  set(preset: number): void {
    this.current = preset;
    const p = PRESETS[preset];
    this.button.replaceChildren();
    const label = document.createElement('span');
    label.className = 'k';
    label.textContent = 'MAP';
    const name = document.createElement('b');
    name.textContent = p?.name ?? `#${preset}`;
    const caret = document.createElement('span');
    caret.className = 'caret';
    caret.textContent = '▾';
    this.button.append(label, name, caret);
    this.button.title = p ? `${p.blurb}\n${p.challenge}` : '';
    this.cards.forEach((c, i) => c.setAttribute('aria-selected', String(i === preset)));
  }

  setOpen(open: boolean): void {
    this.menu.hidden = !open;
    this.button.setAttribute('aria-expanded', String(open));
    if (open) this.cards[this.current]?.focus();
  }

  private card(p: PresetInfo): HTMLButtonElement {
    const c = document.createElement('button');
    c.className = 'card';
    c.setAttribute('role', 'option');
    const swatch = document.createElement('span');
    swatch.className = 'swatch';
    const hex = (v: number) => `#${v.toString(16).padStart(6, '0')}`;
    const pal = p.palette;
    swatch.style.background = `linear-gradient(135deg, ${hex(pal.water ?? pal.low)} 0 25%, ${hex(pal.low)} 25% 50%, ${hex(pal.mid)} 50% 70%, ${hex(pal.rock)} 70% 88%, ${hex(pal.snow)} 88%)`;
    const text = document.createElement('span');
    text.className = 'txt';
    const name = document.createElement('b');
    name.textContent = p.name;
    const blurb = document.createElement('span');
    blurb.className = 'blurb';
    blurb.textContent = p.blurb;
    const challenge = document.createElement('span');
    challenge.className = 'challenge';
    challenge.textContent = p.challenge;
    text.append(name, blurb, challenge);
    c.append(swatch, text);
    c.addEventListener('click', () => {
      this.setOpen(false);
      if (p.id !== this.current) this.onPick(p.id);
    });
    this.cards.push(c);
    return c;
  }
}
