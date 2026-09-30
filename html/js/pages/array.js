// Pocket SDR Web UI - Array page

const MODES = ['Both', 'Bias', 'Att'];
const ALGORITHMS = ['Continuous', 'Static'];
const SOURCES = ['None', 'Continuous', 'Static', 'Loaded', 'External'];

export class ArrayPage {
    constructor(app) {
        this.app = app;
        this.stat = null;
        this.el = document.createElement('div');
        this.el.innerHTML =
            `<div class="toolbar">` +
            `<span class="mono" id="ar-stat">CALIB: ---</span>` +
            `<span class="space"></span>` +
            `<label>Algorithm</label><select id="ar-alg">` +
            ALGORITHMS.map(a => `<option>${a}</option>`).join('') +
            `</select>` +
            `<label>Mode</label><select id="ar-mode">` +
            MODES.map(m => `<option>${m}</option>`).join('') + `</select>` +
            `<button id="ar-run">Start</button>` +
            `<button id="ar-clear">Clear</button>` +
            `<button id="ar-load">Load</button>` +
            `<button id="ar-save">Save</button>` +
            `<label>Geometry file</label><input id="ar-geom" ` +
            `placeholder="array.geom">` +
            `<button id="ar-geom-load">Load geometry</button>` +
            `</div>` +
            `<div class="ar-body" id="ar-body">` +
            `<div class="ar-frame"><div class="ar-title">RF CH DELAY</div>` +
            `<div class="mono" id="ar-bias">---</div></div>` +
            `<div class="ar-frame"><div class="ar-title">ARRAY ATTITUDE` +
            `</div><div class="mono" id="ar-att">---</div></div>` +
            `<div class="ar-frame" id="ar-beam-frame"><div class="ar-title">` +
            `ARRAY CH BEAM DIRECTION</div><div id="ar-beams"></div></div>` +
            `</div>` +
            `<div class="ar-none" id="ar-none">No array channels ` +
            `(start pocket_trk with -ARCH option).</div>`;
        this.el.querySelector('#ar-mode').onchange = () => {
            this.app.ws.send({cmd: 'array_mode',
                mode: MODES.indexOf(this.el.querySelector('#ar-mode').value)});
        };
        this.el.querySelector('#ar-alg').onchange = () => {
            this.app.ws.send({cmd: 'array_alg',
                alg: ALGORITHMS.indexOf(this.el.querySelector('#ar-alg').value)});
        };
        this.el.querySelector('#ar-run').onclick = () => {
            const run = this.stat && this.stat.run ? 0 : 1;
            this.app.ws.send({cmd: 'array_run', run: run});
        };
        this.el.querySelector('#ar-clear').onclick = () => {
            this.app.ws.send({cmd: 'array_run', run: 2});
        };
        this.el.querySelector('#ar-load').onclick = () => {
            this.app.ws.send({cmd: 'array_load'});
        };
        this.el.querySelector('#ar-save').onclick = () => {
            this.app.ws.send({cmd: 'array_save'});
        };
        this.el.querySelector('#ar-geom-load').onclick = () => {
            this.app.ws.send({cmd: 'array_geom',
                file: this.el.querySelector('#ar-geom').value});
        };
        app.ws.on('array_stat', (msg) => this.update(msg));
        app.ws.on('cfg', (msg) => {
            if (this.active && document.activeElement !=
                this.el.querySelector('#ar-geom')) {
                this.el.querySelector('#ar-geom').value = msg.geom || '';
            }
        });
        app.ws.on('ack', (msg) => { // refresh promptly after a command
            if (this.active && msg.cmd && msg.cmd.startsWith('array_')) {
                this.app.ws.get('array_stat');
            }
        });
    }
    update(msg) {
        if (!this.active) return;
        this.stat = msg;
        const none = msg.has_array === false ||
            (msg.has_array === undefined && msg.narch <= 0);
        this.el.querySelector('#ar-body').style.display = none ? 'none' : '';
        this.el.querySelector('#ar-none').style.display = none ? '' : 'none';
        if (none) {
            this.el.querySelector('#ar-stat').textContent = 'CALIB: ---';
            return;
        }
        const stat = this.el.querySelector('#ar-stat');
        const alg = this.el.querySelector('#ar-alg');
        const run = this.el.querySelector('#ar-run');
        if (msg.alg !== 0 && msg.alg !== 1) {
            stat.textContent = 'CALIB: Server update required';
            stat.classList.add('warn-txt');
            alg.value = '';
            alg.disabled = true;
            run.disabled = true;
            return;
        }
        alg.disabled = false;
        run.disabled = false;
        const source = SOURCES[msg.source] || 'None';
        const state = msg.run ? (msg.alg === 1 ? 'COLLECTING' : 'RUN') :
            (msg.valid ? `VALID (${source})` : 'INVALID');
        const sc = msg.static || {};
        stat.textContent = msg.alg === 1 ?
            `CALIB: ${state}  EPOCHS: ${sc.epochs || 0}/${sc.total_epochs || 0}` +
            `  MEAS: ${sc.meas || 0}  SATS: ${sc.sats || 0}` +
            `  RMS: ${msg.valid ? msg.rms.toFixed(4) :
                (sc.last_rms == null ? '---' : sc.last_rms.toFixed(4))} m` :
            `CALIB: ${state}  EPOCHS: ${msg.nep}  ` +
            `RMS: ${msg.rms.toFixed(4)} m`;
        stat.classList.toggle('warn-txt', !!msg.run || !msg.valid);
        run.textContent = msg.run ? 'Stop' : 'Start';
        const sel = this.el.querySelector('#ar-mode');
        if (document.activeElement != sel) sel.value = MODES[msg.mode] ||
            'Both';
        if (document.activeElement != alg) alg.value =
            ALGORITHMS[msg.alg];
        this.el.querySelector('#ar-beam-frame').style.display =
            msg.narch > 0 ? '' : 'none';
        this.el.querySelector('#ar-bias').innerHTML = msg.bias.map(
            (b, i) => `CH${i+1}: ${b.toFixed(4)} m`).join('&nbsp;&nbsp; ');
        this.el.querySelector('#ar-att').textContent =
            `ROLL: ${msg.rpy[0].toFixed(3)}°  ` +
            `PITCH: ${msg.rpy[1].toFixed(3)}°  ` +
            `YAW: ${msg.rpy[2].toFixed(3)}°`;
        const beams = this.el.querySelector('#ar-beams');
        if (beams.children.length != msg.beams.length) {
            beams.innerHTML = msg.beams.map(b =>
                `<div class="ar-beam" data-ch="${b.ch}">` +
                `<span class="mono">CH${b.ch}</span>` +
                `<label>AZ</label><input type="number" class="az" ` +
                `min="0" max="360" step="1">` + `<label>°</label>` +
                `<label>EL</label><input type="number" class="el" ` +
                `min="0" max="90" step="1">` + `<label>°</label>` +
                `<button>Update</button></div>`).join('');
            for (const row of beams.children) {
                row.querySelector('button').onclick = () => {
                    this.app.ws.send({cmd: 'array_beam',
                        rfch: parseInt(row.dataset.ch),
                        az: parseFloat(row.querySelector('.az').value) || 0,
                        el: parseFloat(row.querySelector('.el').value) || 0});
                };
            }
        }
        msg.beams.forEach((b, i) => {
            const row = beams.children[i];
            const az = row.querySelector('.az'), el = row.querySelector('.el');
            if (document.activeElement != az && document.activeElement != el) {
                az.value = b.az.toFixed(1);
                el.value = b.el.toFixed(1);
            }
        });
        for (const e of this.el.querySelectorAll('#ar-bias, #ar-att')) {
            e.classList.toggle('warn-txt', !!msg.run);
        }
    }
    show() {
        this.active = true;
        this.app.ws.sub('array_stat', {cyc: 500});
        this.app.ws.get('cfg');
    }
    hide() {
        this.active = false;
        this.app.ws.unsub('array_stat');
    }
}
