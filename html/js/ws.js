// Pocket SDR Web UI - WebSocket client (protocol: doc/design_web_ui.md)

// decode binary frame ---------------------------------------------------------
export function decodeFrame(buf) {
    const dv = new DataView(buf);
    const type = dv.getUint8(0);
    if (type == 1) { // PSD
        const n = dv.getUint32(12, true);
        return {
            type: 'psd', rfch: dv.getUint8(1), IQ: dv.getUint8(2),
            bits: dv.getUint8(3), fs: dv.getFloat32(4, true),
            tave: dv.getFloat32(8, true), n: n, fo: dv.getFloat64(16, true),
            psd: new Float32Array(buf, 24, n)
        };
    }
    else if (type == 2) { // correlator snapshot
        const n = dv.getUint32(4, true);
        let off = 40;
        const pos = new Float32Array(buf, off, n); off += 4 * n;
        const C = new Float32Array(buf, off, 2 * n); off += 8 * n;
        const aveP = new Float32Array(buf, off, n); off += 4 * n;
        const aveI = new Float32Array(buf, off, n);
        return {
            type: 'corr', state: dv.getUint8(1), ch: dv.getUint16(2, true),
            n: n, npos: dv.getUint32(8, true), fs: dv.getFloat32(12, true),
            lock: dv.getFloat32(16, true), cn0: dv.getFloat32(20, true),
            fd: dv.getFloat32(24, true), time: dv.getFloat32(28, true),
            coff: dv.getFloat64(32, true), pos: pos, C: C, aveP: aveP,
            aveI: aveI
        };
    }
    else if (type == 3) { // correlator history
        const n = dv.getUint32(4, true);
        return {
            type: 'corr_hist', ch: dv.getUint16(2, true), n: n,
            time: dv.getFloat64(8, true), T: dv.getFloat32(16, true),
            P: new Float32Array(buf, 24, 2 * n)
        };
    }
    else if (type == 4) { // spatial power map, azimuth varies fastest
        const naz = dv.getUint16(20, true), nel = dv.getUint16(22, true);
        if (buf.byteLength != 40 + 4 * naz * nel) return null;
        return {
            type: 'spatial', alg: dv.getUint8(1), ch: dv.getUint16(2, true),
            seq: dv.getUint32(4, true), time: dv.getFloat64(8, true),
            cn0: dv.getFloat32(16, true), naz, nel,
            az0: dv.getFloat32(24, true), el0: dv.getFloat32(28, true),
            daz: dv.getFloat32(32, true), del: dv.getFloat32(36, true),
            power: new Float32Array(buf, 40, naz * nel)
        };
    }
    return null;
}

// WebSocket client with auto-reconnect and subscription replay ----------------
export class WsClient {
    constructor() {
        this.handlers = {};
        this.subs = {};
        this.ws = null;
        this.connected = false;
        this.retry = 500;
    }
    connect() {
        const url = (location.protocol == 'https:' ? 'wss://' : 'ws://') +
            location.host + '/ws';
        this.ws = new WebSocket(url);
        this.ws.binaryType = 'arraybuffer';
        this.ws.onopen = () => {
            this.connected = true;
            this.retry = 500;
            this.emit('open', {});
            for (const topic in this.subs) {
                this.send(Object.assign({cmd: 'sub', topic: topic},
                    this.subs[topic]));
            }
        };
        this.ws.onmessage = (ev) => {
            if (typeof ev.data == 'string') {
                let msg = null;
                try { msg = JSON.parse(ev.data); } catch (e) { return; }
                if (msg && msg.type) this.emit(msg.type, msg);
            }
            else {
                const msg = decodeFrame(ev.data);
                if (msg) this.emit(msg.type, msg);
            }
        };
        this.ws.onclose = () => {
            this.connected = false;
            this.emit('close', {});
            setTimeout(() => this.connect(), this.retry);
            this.retry = Math.min(this.retry * 2, 5000);
        };
    }
    on(type, fn) {
        (this.handlers[type] = this.handlers[type] || []).push(fn);
    }
    emit(type, msg) {
        for (const fn of this.handlers[type] || []) fn(msg);
    }
    send(obj) {
        if (this.connected) this.ws.send(JSON.stringify(obj));
    }
    sub(topic, params) {
        this.subs[topic] = params || {};
        this.send(Object.assign({cmd: 'sub', topic: topic}, this.subs[topic]));
    }
    unsub(topic) {
        delete this.subs[topic];
        this.send({cmd: 'unsub', topic: topic});
    }
    get(topic, params) {
        this.send(Object.assign({cmd: 'get', topic: topic}, params || {}));
    }
}
