// Pure edits on a dashboards document. Each returns a new document or a message; the input is never changed.
import { CAPS, normalizeCard, newId, type Card, type Dashboard, type DashboardsDoc, type Section } from "./dashboardModel";

export type OpResult = { ok: true; doc: DashboardsDoc } | { ok: false; message: string };
export type CardInput = Partial<Omit<Card, "id" | "type">> & { type: Card["type"] };

const fail = (message: string): OpResult => ({ ok: false, message });
const NAME_MAX = 40;
const TITLE_MAX = 60;

const totalCards = (doc: DashboardsDoc) => doc.dashboards.reduce((n, d) => n + d.sections.reduce((m, s) => m + s.cards.length, 0), 0);
const allIds = (doc: DashboardsDoc) => new Set(doc.dashboards.flatMap((d) => [d.id, ...d.sections.flatMap((s) => [s.id, ...s.cards.map((c) => c.id)])]));
const insertAt = <T>(list: T[], item: T, index: number): T[] => {
  const i = Number.isInteger(index) ? Math.min(Math.max(index, 0), list.length) : list.length;
  return [...list.slice(0, i), item, ...list.slice(i)];
};

function withDashboard(doc: DashboardsDoc, did: string, fn: (d: Dashboard) => Dashboard | string): OpResult {
  const i = doc.dashboards.findIndex((d) => d.id === did);
  if (i < 0) return fail("Dashboard not found.");
  const next = fn(doc.dashboards[i]);
  if (typeof next === "string") return fail(next);
  return { ok: true, doc: { ...doc, dashboards: doc.dashboards.map((d, j) => (j === i ? next : d)) } };
}

function withSection(doc: DashboardsDoc, did: string, sid: string, fn: (s: Section) => Section | string): OpResult {
  return withDashboard(doc, did, (d) => {
    const i = d.sections.findIndex((s) => s.id === sid);
    if (i < 0) return "Section not found.";
    const next = fn(d.sections[i]);
    if (typeof next === "string") return next;
    return { ...d, sections: d.sections.map((s, j) => (j === i ? next : s)) };
  });
}

const cardsIn = (sections: Section[]) => sections.reduce((n, s) => n + s.cards.length, 0);

export function addDashboard(doc: DashboardsDoc, name: string, sections: Section[] = []): OpResult {
  if (doc.dashboards.length >= CAPS.dashboards) return fail(`At most ${CAPS.dashboards} dashboards.`);
  const trimmed = name.trim().slice(0, NAME_MAX);
  if (!trimmed) return fail("Enter a name.");
  if (totalCards(doc) + cardsIn(sections) > CAPS.cardsTotal) return fail(`At most ${CAPS.cardsTotal} cards in total.`);
  return { ok: true, doc: { ...doc, dashboards: [...doc.dashboards, { id: newId(), name: trimmed, sections }] } };
}

/** Replaces the sections of one dashboard. Its id and name stay; the other dashboards are not touched. */
export function resetDashboard(doc: DashboardsDoc, did: string, sections: Section[]): OpResult {
  return withDashboard(doc, did, (d) => {
    const others = totalCards(doc) - cardsIn(d.sections);
    if (others + cardsIn(sections) > CAPS.cardsTotal) return `At most ${CAPS.cardsTotal} cards in total.`;
    return { ...d, sections };
  });
}

export function renameDashboard(doc: DashboardsDoc, did: string, name: string): OpResult {
  const trimmed = name.trim().slice(0, NAME_MAX);
  if (!trimmed) return fail("Enter a name.");
  return withDashboard(doc, did, (d) => ({ ...d, name: trimmed }));
}

export function removeDashboard(doc: DashboardsDoc, did: string): OpResult {
  if (doc.dashboards.length <= 1) return fail("Keep at least one dashboard.");
  if (!doc.dashboards.some((d) => d.id === did)) return fail("Dashboard not found.");
  return { ok: true, doc: { ...doc, dashboards: doc.dashboards.filter((d) => d.id !== did) } };
}

export function addSection(doc: DashboardsDoc, did: string, title = ""): OpResult {
  return withDashboard(doc, did, (d) =>
    d.sections.length >= CAPS.sectionsPerDashboard
      ? `At most ${CAPS.sectionsPerDashboard} sections per dashboard.`
      : { ...d, sections: [...d.sections, { id: newId(), title: title.trim().slice(0, TITLE_MAX), cards: [] }] });
}

export const renameSection = (doc: DashboardsDoc, did: string, sid: string, title: string): OpResult =>
  withSection(doc, did, sid, (s) => ({ ...s, title: title.trim().slice(0, TITLE_MAX) }));

export const removeSection = (doc: DashboardsDoc, did: string, sid: string): OpResult =>
  withDashboard(doc, did, (d) => (d.sections.some((s) => s.id === sid) ? { ...d, sections: d.sections.filter((s) => s.id !== sid) } : "Section not found."));

export const moveSection = (doc: DashboardsDoc, did: string, sid: string, toIndex: number): OpResult =>
  withDashboard(doc, did, (d) => {
    const s = d.sections.find((x) => x.id === sid);
    if (!s) return "Section not found.";
    return { ...d, sections: insertAt(d.sections.filter((x) => x.id !== sid), s, toIndex) };
  });

function buildCard(doc: DashboardsDoc, input: CardInput, id?: string): Card | null {
  const taken = allIds(doc);
  if (id) taken.delete(id);
  const card = normalizeCard({ ...input, id }, taken);
  return card;
}

export function addCard(doc: DashboardsDoc, did: string, sid: string, input: CardInput): OpResult {
  return withSection(doc, did, sid, (s) => {
    if (s.cards.length >= CAPS.cardsPerSection) return `At most ${CAPS.cardsPerSection} cards per section.`;
    if (totalCards(doc) >= CAPS.cardsTotal) return `At most ${CAPS.cardsTotal} cards in total.`;
    const card = buildCard(doc, input);
    if (!card) return "That card is not valid.";
    return { ...s, cards: [...s.cards, card] };
  });
}

function locate(doc: DashboardsDoc, did: string, cid: string): { sid: string; index: number } | null {
  const d = doc.dashboards.find((x) => x.id === did);
  for (const s of d?.sections ?? []) {
    const index = s.cards.findIndex((c) => c.id === cid);
    if (index >= 0) return { sid: s.id, index };
  }
  return null;
}

export function updateCard(doc: DashboardsDoc, did: string, cid: string, input: CardInput): OpResult {
  const at = locate(doc, did, cid);
  if (!at) return fail("Card not found.");
  return withSection(doc, did, at.sid, (s) => {
    const card = buildCard(doc, input, cid);
    if (!card) return "That card is not valid.";
    return { ...s, cards: s.cards.map((c, i) => (i === at.index ? card : c)) };
  });
}

export const removeCard = (doc: DashboardsDoc, did: string, cid: string): OpResult => {
  const at = locate(doc, did, cid);
  if (!at) return fail("Card not found.");
  return withSection(doc, did, at.sid, (s) => ({ ...s, cards: s.cards.filter((c) => c.id !== cid) }));
};

export function duplicateCard(doc: DashboardsDoc, did: string, cid: string): OpResult {
  const at = locate(doc, did, cid);
  if (!at) return fail("Card not found.");
  return withSection(doc, did, at.sid, (s) => {
    if (s.cards.length >= CAPS.cardsPerSection) return `At most ${CAPS.cardsPerSection} cards per section.`;
    if (totalCards(doc) >= CAPS.cardsTotal) return `At most ${CAPS.cardsTotal} cards in total.`;
    return { ...s, cards: insertAt(s.cards, { ...s.cards[at.index], pids: [...s.cards[at.index].pids], id: newId() }, at.index + 1) };
  });
}

/** Moves a card to a section at an index (clamped). Within one section the total stays, so only a different section can hit the cap. */
export function moveCard(doc: DashboardsDoc, did: string, cid: string, toSid: string, toIndex: number): OpResult {
  const at = locate(doc, did, cid);
  if (!at) return fail("Card not found.");
  const d = doc.dashboards.find((x) => x.id === did)!;
  const target = d.sections.find((s) => s.id === toSid);
  if (!target) return fail("Section not found.");
  if (toSid !== at.sid && target.cards.length >= CAPS.cardsPerSection) return fail(`At most ${CAPS.cardsPerSection} cards per section.`);
  const card = d.sections.flatMap((s) => s.cards).find((c) => c.id === cid)!;
  return withDashboard(doc, did, (dd) => ({
    ...dd,
    sections: dd.sections.map((s) => {
      const without = s.cards.filter((c) => c.id !== cid);
      return s.id === toSid ? { ...s, cards: insertAt(without, card, toIndex) } : { ...s, cards: without };
    }),
  }));
}
