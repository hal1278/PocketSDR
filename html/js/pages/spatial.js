// Pocket SDR Web UI - selected-signal spatial heatmap

export class SpatialPage {
    constructor(app) {
        this.app = app;
        this.active = false;
        this.ch = 0;
        this.signals = [];
        this.map = null;
        this.los = null;
        this.el = document.createElement('div');
        this.el.innerHTML = `<div class="toolbar">` +
            `<label>Signal</label><select id="sp-signal"></select>` +
            `<label>Algorithm</label><select id="sp-alg">` +
            `<option>Bartlett</option></select>` +
            `<span class="space"></span>` +
            `<span class="mono" id="sp-calib">Nominal array</span>` +
            `<span class="mono" id="sp-state">Waiting for receiver</span>` +
            `</div><div class="sp-body">` +
            `<canvas id="sp-map"></canvas>` +
            `<div class="sp-scale">Relative power: 0 to −25 dB; ` +
            `brightest cell is 0 dB. ○ Predicted satellite LOS.</div>` +
            `</div>`;
        this.canvas = this.el.querySelector('#sp-map');
        this.el.querySelector('#sp-signal').onchange = () => {
            this.ch = Number(this.el.querySelector('#sp-signal').value);
            this.select();
        };
        this.el.querySelector('#sp-alg').onchange = () => this.select();
        app.ws.on('ch_stat', msg => this.updateSignals(msg));
        app.ws.on('sat_stat', msg => this.updateLos(msg));
        app.ws.on('array_stat', msg => {
            if (!this.active) return;
            const source = ['None', 'Continuous', 'Static', 'Loaded',
                'External'][msg.source] || 'Unknown';
            this.el.querySelector('#sp-calib').textContent = msg.valid ?
                `Calibrated (${source})` : 'Nominal array · direction uncalibrated';
        });
        app.ws.on('spatial', msg => {
            if (!this.active || msg.ch != this.ch) return;
            this.map = msg;
            this.el.querySelector('#sp-state').textContent =
                `LOCK  C/N0 ${msg.cn0.toFixed(1)} dB-Hz  ` +
                `t=${msg.time.toFixed(3)} s  #${msg.seq}`;
            this.redraw();
        });
        app.ws.on('hello', msg => {
            if (!msg.run) {
                this.map = null;
                this.los = null;
                this.el.querySelector('#sp-state').textContent =
                    'Receiver stopped';
                if (this.active) this.redraw();
            }
            else if (this.active && this.ch) this.select();
        });
    }
    updateSignals(msg) {
        if (!this.active) return;
        const signals = [];
        for (const line of msg.str.split('\n').slice(2)) {
            const f = line.trim().split(/\s+/);
            if (f.length < 16 || f[3] != 'L1CA' ||
                Number(f[1]) > this.app.info.nrfch || Number(f[5]) < 2) continue;
            signals.push({ch: Number(f[0]), sat: f[2], sig: f[3],
                cn0: Number(f[6])});
        }
        this.signals = signals;
        const sel = this.el.querySelector('#sp-signal');
        const prior = this.ch;
        if (!signals.some(s => s.ch == this.ch)) {
            this.ch = signals.length ? signals.reduce((a, b) =>
                a.cn0 >= b.cn0 ? a : b).ch : 0;
        }
        const key = signals.map(s => s.ch).join(',');
        if (key != this.listKey) {
            this.listKey = key;
            sel.innerHTML = signals.length ? signals.map(s =>
                `<option value="${s.ch}">${s.sat} ${s.sig} · CH${s.ch}` +
                ` · ${s.cn0.toFixed(1)} dB-Hz</option>`).join('') :
                '<option value="0">No locked L1 C/A signal</option>';
        }
        sel.value = String(this.ch);
        if (this.ch != prior) this.select();
        if (!this.ch) {
            this.el.querySelector('#sp-state').textContent =
                'Waiting for a locked L1 C/A signal';
        }
    }
    select() {
        this.map = null;
        this.los = null;
        this.redraw();
        this.app.ws.send({cmd: 'spatial_select', ch: this.ch,
            alg: this.el.querySelector('#sp-alg').value});
        const sig = this.signals.find(s => s.ch == this.ch);
        if (sig) this.app.ws.sub('sat_stat', {sats: sig.sat, cyc: 500});
        else this.app.ws.unsub('sat_stat');
        if (this.ch) this.el.querySelector('#sp-state').textContent =
            'Accumulating spatial snapshots';
    }
    updateLos(msg) {
        if (!this.active) return;
        const sig = this.signals.find(s => s.ch == this.ch);
        const sat = sig && msg.sats.find(s => s.sat == sig.sat);
        this.los = sat && sat.eph && sat.pvt && sat.el >= 0 ? sat : null;
        this.redraw();
    }
    redraw() {
        const canvas = this.canvas;
        const width = Math.max(400, canvas.clientWidth || 800);
        const height = Math.max(220, canvas.clientHeight || 400);
        const ratio = window.devicePixelRatio || 1;
        canvas.width = Math.round(width * ratio);
        canvas.height = Math.round(height * ratio);
        const ctx = canvas.getContext('2d');
        ctx.scale(ratio, ratio);
        ctx.fillStyle = '#101c2b';
        ctx.fillRect(0, 0, width, height);
        const x0 = 44, y0 = 12, w = width - 59, h = height - 44;
        const map = this.map;
        if (map) {
            let peak = 0;
            for (const p of map.power) if (p > peak) peak = p;
            for (let el = 0; el < map.nel; el++) {
                for (let az = 0; az < map.naz; az++) {
                    const p = map.power[el * map.naz + az];
                    const db = 10 * Math.log10(Math.max(p, 1e-30) /
                        Math.max(peak, 1e-30));
                    const t = Math.max(0, Math.min(1, (db + 25) / 25));
                    const hue = 235 - 230 * t;
                    ctx.fillStyle = `hsl(${hue}, 90%, ${15 + 52 * t}%)`;
                    ctx.fillRect(x0 + az * w / map.naz,
                        y0 + (map.nel - 1 - el) * h / map.nel,
                        Math.ceil(w / map.naz) + 1,
                        Math.ceil(h / map.nel) + 1);
                }
            }
        }
        ctx.strokeStyle = '#b9c4d0';
        ctx.fillStyle = '#d6dee6';
        ctx.font = '11px sans-serif';
        ctx.strokeRect(x0, y0, w, h);
        const az0 = map ? map.az0 : 0;
        const azSpan = map ? map.naz * map.daz : 360;
        const el0 = map ? map.el0 : 0;
        const elSpan = map ? (map.nel - 1) * map.del : 90;
        for (let i = 0; i <= 4; i++) {
            const x = x0 + i / 4 * w;
            ctx.fillText(`${(az0 + i / 4 * azSpan).toFixed(0)}°`,
                x - 12, y0 + h + 19);
        }
        for (let i = 0; i <= 3; i++) {
            const y = y0 + (1 - i / 3) * h;
            ctx.fillText(`${(el0 + i / 3 * elSpan).toFixed(0)}°`,
                7, y + 4);
        }
        if (this.los) {
            const x = x0 + (this.los.az - az0) / azSpan * w;
            const y = y0 + (1 - (this.los.el - el0) / elSpan) * h;
            if (x < x0 || x > x0 + w || y < y0 || y > y0 + h) return;
            ctx.beginPath();
            ctx.arc(x, y, 7, 0, 2 * Math.PI);
            ctx.strokeStyle = '#fff';
            ctx.lineWidth = 2;
            ctx.stroke();
        }
    }
    show() {
        this.active = true;
        this.app.ws.sub('ch_stat', {chno: 0, min_lock: 2, rfch: 0,
            cyc: 500});
        this.app.ws.sub('spatial', {cyc: 250});
        this.app.ws.sub('array_stat', {cyc: 500});
        if (this.ch) this.select();
        this.redraw();
    }
    hide() {
        this.active = false;
        this.app.ws.unsub('ch_stat');
        this.app.ws.unsub('sat_stat');
        this.app.ws.unsub('spatial');
        this.app.ws.unsub('array_stat');
    }
}
