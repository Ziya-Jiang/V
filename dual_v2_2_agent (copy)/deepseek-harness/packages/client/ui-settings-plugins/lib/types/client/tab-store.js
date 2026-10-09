/**
 * The configurable-plugins tab's card list.
 *
 * The tab dispatches its slot by settings namespace, so what it renders is
 * the intersection of two ledgers: the namespaces the Host serves and the
 * cards registered into `settings.plugin.item`. A served namespace no card
 * claims renders nothing — another surface owns it, or this deployment ships
 * no browser half for it — and a card whose namespace the Host does not serve
 * is never dispatched, so a plugin this deployment did not compose leaves no
 * trace and does not count toward the empty line.
 */
import { createSnapshotStore } from '@deepseek-ai/dsh-client-runtime/client';
/** Reads the served namespaces and pairs them with the cards that claim them. */
export class ConfigurablePluginsTabController {
    api;
    entries;
    store = createSnapshotStore({ loaded: false, namespaces: [] });
    /** Last Host answer; kept so a slot mutation republishes without a wire read. */
    served = [];
    loaded = false;
    generation = 0;
    disposed = false;
    /**
     * @param api - settings wire face.
     * @param entries - reads the cards currently registered into the section's slot.
     */
    constructor(api, entries) {
        this.api = api;
        this.entries = entries;
    }
    /** Opaque read of {@link disposed}: control flow cannot narrow it across awaits. */
    isDisposed() {
        return this.disposed;
    }
    /**
     * Re-read the served namespaces from the Host and republish.
     * @returns settlement after the read, or immediately once disposed.
     */
    async load() {
        if (this.isDisposed())
            return;
        const generation = ++this.generation;
        let response;
        try {
            response = await this.api.settings.describe({});
        }
        catch (_settingsReadFailure) {
            // The tab keeps the namespaces it last knew; the next invalidation
            // or reconnect reads again.
            return;
        }
        if (this.isDisposed() || generation !== this.generation || !response.result.ok)
            return;
        this.served = response.result.value.namespaces.map(view => view.ns);
        this.loaded = true;
        this.publish();
    }
    /** Republish after the slot ledger changed; a card registered late joins here. */
    refresh() {
        if (this.disposed)
            return;
        this.publish();
    }
    /** Stop publishing; an in-flight read settles without touching the store. */
    dispose() {
        this.disposed = true;
        this.generation += 1;
    }
    /**
     * Build the face the tab's slot registration injects.
     * @returns the tab's snapshot source.
     */
    inject() {
        return { hooks: { configurablePlugins: this.store } };
    }
    publish() {
        const served = new Set(this.served);
        const namespaces = this.entries().flatMap(entry => entry.options.key !== undefined && served.has(entry.options.key) ? [entry.options.key] : []);
        const previous = this.store.getSnapshot();
        // Every settings-document commit re-reads, and most of them change nothing
        // this section shows. An observable source must keep its snapshot
        // reference until the fact moves, or each unrelated save re-renders the
        // whole card list (packages/client/AGENTS.md reactive rule 5).
        if (previous.loaded === this.loaded
            && previous.namespaces.length === namespaces.length
            && previous.namespaces.every((ns, index) => ns === namespaces[index]))
            return;
        this.store.set({ loaded: this.loaded, namespaces });
    }
}
//# sourceMappingURL=tab-store.js.map