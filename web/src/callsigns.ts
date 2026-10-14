// Radio callsigns shown in the console, tooltip and event log.

import { OPERATOR_ID } from './chatter';

/** The WiFi station the mesh root bridges to (telemetry.ts STATION_ID; chatter UPLINK dst). */
const WIFI_STATION_ID = 0xfffd;

const NATO = [
  'ALPHA', 'BRAVO', 'CHARLIE', 'DELTA', 'ECHO', 'FOXTROT', 'GOLF', 'HOTEL', 'INDIA',
  'JULIETT', 'KILO', 'LIMA', 'MIKE', 'NOVEMBER', 'OSCAR', 'PAPA', 'QUEBEC', 'ROMEO',
  'SIERRA', 'TANGO', 'UNIFORM', 'VICTOR', 'WHISKEY', 'XRAY', 'YANKEE', 'ZULU',
] as const;

/** Callsigns are asked for on every console line; cache them once built. */
const cache: string[] = [];

/**
 * Drone 0 (the base) is OVERWATCH; others ALPHA-1 … ZULU-1, ALPHA-2, ….
 * The operator console's sender id (chatter OPERATOR_ID) is OPERATOR, and the
 * WiFi station the base bridges telemetry to is WIFI-STA.
 */
export function callsign(id: number): string {
  const cached = cache[id];
  if (cached !== undefined) return cached;
  if (id === OPERATOR_ID) return 'OPERATOR';
  if (id === WIFI_STATION_ID) return 'WIFI-STA';
  let name: string;
  if (id === 0) name = 'OVERWATCH';
  else if (!Number.isInteger(id) || id < 0) name = `#${id}`;
  else name = `${NATO[(id - 1) % 26]}-${1 + Math.floor((id - 1) / 26)}`;
  if (Number.isInteger(id) && id >= 0 && id < 4096) cache[id] = name;
  return name;
}

/**
 * The drone id a search string names, or -1: a callsign ("bravo-2", "BRAVO 2",
 * "bravo" = BRAVO-1), "overwatch" / "base", or a bare id ("7", "#7", "d7").
 */
export function parseCallsign(text: string): number {
  const s = text.trim().toUpperCase();
  if (s === '') return -1;
  if (s === 'OVERWATCH' || s === 'BASE') return 0;
  const num = /^[#D]?(\d+)$/.exec(s);
  if (num) return Number(num[1]);
  const m = /^([A-Z]+)(?:[\s-]*(\d+))?$/.exec(s);
  if (!m) return -1;
  const letter = NATO.findIndex((n) => n === m[1] || (m[1] === 'X-RAY' && n === 'XRAY'));
  if (letter < 0) return -1;
  const round = m[2] ? Number(m[2]) : 1;
  return round >= 1 ? letter + 1 + (round - 1) * 26 : -1;
}
