// Dashboard state without Svelte: the committed document, the draft edit copy and persistence.
// Mutators return an error message or null. dashboardDocStore.svelte.ts wraps this reactively.
import { sectionsFromTemplate, type Dashboard, type DashboardsDoc } from "./dashboardModel";
import * as ops from "./dashboardOps";
import type { DashboardTemplate } from "./dashboardTemplates";
import type { CardInput, OpResult } from "./dashboardOps";
import type { createDashboardPersistence, SdSave } from "./dashboardPersistence";

export interface DashboardView {
  dashboards: Dashboard[];
  active: Dashboard;
  editing: boolean;
  /** Nothing has been stored or chosen yet: the page offers starting layouts instead of showing the default. */
  fresh: boolean;
  /** The SD copy is being read after login. */
  loading: boolean;
  /** The SD copy exists but could not be read; the page shows the local copy. */
  unreadable: boolean;
  /** Result of the last Done: where it was saved. */
  save: { sd: SdSave | "idle"; message?: string };
}

export interface ControllerDeps {
  persistence: ReturnType<typeof createDashboardPersistence>;
  isLoggedIn(): boolean;
  now(): number;
  onChange(): void;
}

export function createDashboardController(deps: ControllerDeps) {
  const first = deps.persistence.initial();
  let committed: DashboardsDoc = first.doc;
  let fresh = first.source === "default";
  let loading = false;
  let unreadable = false;
  let draft: DashboardsDoc | null = null;
  let activeId = committed.dashboards[0].id;
  let save: DashboardView["save"] = { sd: "idle" };

  const doc = () => draft ?? committed;
  const activeOf = (d: DashboardsDoc) => d.dashboards.find((x) => x.id === activeId) ?? d.dashboards[0];
  const changed = () => deps.onChange();

  function view(): DashboardView {
    const d = doc();
    return { dashboards: d.dashboards, active: activeOf(d), editing: draft !== null, fresh, loading, unreadable, save };
  }

  function apply(run: (d: DashboardsDoc) => OpResult): string | null {
    if (!draft) return "Not in edit mode.";
    const r = run(draft);
    if (!r.ok) return r.message;
    draft = r.doc;
    activeId = activeOf(draft).id;
    changed();
    return null;
  }

  const inActive = (run: (d: DashboardsDoc, did: string) => OpResult) => apply((d) => run(d, activeOf(d).id));

  function startEdit() {
    if (draft) return;
    draft = committed;
    changed();
  }

  function cancel() {
    if (!draft) return;
    draft = null;
    activeId = activeOf(committed).id;
    changed();
  }

  /** Commits the draft, then saves it. The commit stands even when the SD write fails; `view().save` says what happened. */
  async function done(): Promise<void> {
    if (!draft) return;
    committed = { ...draft, updatedAt: deps.now() };
    draft = null;
    fresh = false;
    save = { sd: "idle" };
    changed();
    save = await deps.persistence.save(committed, deps.isLoggedIn());
    changed();
  }

  /** After login: adopts the newest copy. Skipped while editing so a draft is never replaced. */
  async function sync(): Promise<void> {
    if (draft) return;
    loading = true;
    changed();
    const r = await deps.persistence.sync();
    loading = false;
    unreadable = r.unreadable;
    if (draft) {
      changed();
      return;
    }
    committed = r.doc;
    fresh = r.source === "default";
    activeId = activeOf(committed).id;
    changed();
  }

  return {
    view,
    startEdit,
    cancel,
    done,
    sync,
    setActive(id: string) {
      if (doc().dashboards.some((d) => d.id === id)) { activeId = id; changed(); }
    },
    addDashboard: (name: string, template?: DashboardTemplate) => {
      const err = apply((d) => ops.addDashboard(d, name, template ? sectionsFromTemplate(template) : []));
      if (!err && draft) activeId = draft.dashboards[draft.dashboards.length - 1].id;
      return err;
    },
    resetToTemplate: (template: DashboardTemplate) => inActive((d, did) => ops.resetDashboard(d, did, sectionsFromTemplate(template))),
    renameDashboard: (id: string, name: string) => apply((d) => ops.renameDashboard(d, id, name)),
    removeDashboard: (id: string) => {
      const err = apply((d) => ops.removeDashboard(d, id));
      if (!err) { activeId = activeOf(doc()).id; changed(); }
      return err;
    },
    addSection: (title = "") => inActive((d, did) => ops.addSection(d, did, title)),
    renameSection: (sid: string, title: string) => inActive((d, did) => ops.renameSection(d, did, sid, title)),
    removeSection: (sid: string) => inActive((d, did) => ops.removeSection(d, did, sid)),
    moveSection: (sid: string, toIndex: number) => inActive((d, did) => ops.moveSection(d, did, sid, toIndex)),
    addCard: (sid: string, input: CardInput) => inActive((d, did) => ops.addCard(d, did, sid, input)),
    updateCard: (cid: string, input: CardInput) => inActive((d, did) => ops.updateCard(d, did, cid, input)),
    removeCard: (cid: string) => inActive((d, did) => ops.removeCard(d, did, cid)),
    duplicateCard: (cid: string) => inActive((d, did) => ops.duplicateCard(d, did, cid)),
    moveCard: (cid: string, toSid: string, toIndex: number) => inActive((d, did) => ops.moveCard(d, did, cid, toSid, toIndex)),
  };
}
