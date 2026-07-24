# Offline processing mode progress

- 2026-07-22: 作業開始。branch `develop`、開始時 HEAD `5bea7a6b624d2c3af07407c63fb4a196549653da` を確認。
- 2026-07-22: upstream が `tomojitakasu/PocketSDR` であることを確認。repository は現在の checkout を維持する。
- 2026-07-22: 開始前変更を確認。`AGENTS.md`、各種 makefile、`lib/build/*`、`lib/win32/*` の変更、および `.clangd`、`memo/build.md`、`pyproject.toml`、`python/pocket_sdr.ini`、`uv.lock` 等の未追跡ファイルが存在。今回の対象外として保持する。
- 2026-07-22: 添付仕様と `memo/build.md` を確認し、receiver/channel の thread ownership と既存 test 構成を調査中。
- 2026-07-22: 実装方針を確定。offline状態はlocal file start時だけ有効化し、channel threadが初回捕捉・lock loss・sample-time再捕捉期限を所有、receiver mutexで単調read indexとEOF drain状態を同期する。
- 2026-07-22: `src/sdr_rcv.c`、`src/pocket_sdr.h`、`src/sdr_ch.c`、`app/pocket_trk/pocket_trk.c`、`doc/command_ref.md`へ初期実装を追加。次にunit test追加とbuild前レビューを行う。
- 2026-07-22: `test/src/test_sdr_rcv.c`へdummy IF integration testを追加。初回2channel一括捕捉、初回失敗後IDLE、FAST_SRCH無効化、EOF未完了捕捉解放、channel数0、`tscale=0`でwall-clock replay sleepを使わないことを確認。
- 2026-07-22: `test/utest` の全unit testをsourceからbuild・実行し、全test成功。更新後の`test_sdr_rcv`単体再実行も成功。
- 2026-07-22: UCRT64で `lib/build && mingw32-make` が成功。ユーザー許可後、`mingw32-make -f libsdr.mk install`で`lib/win32/libsdr.a/.so`を更新。`libsdr_master.a/.so`のSHA-256不変を確認。
- 2026-07-22: `app/pocket_trk && mingw32-make pocket_trk`のlink成功、生成binary内の`[-offline]` usage文字列を確認。実行は`SoapySDR.dll`不在のため不可。
- 2026-07-22: sampleに実IF dataは存在するが、上記runtime DLL不足のため実信号`pocket_trk -offline`試験は未実行。今回生成したtest/app一時artifactは除去済みで、install済みlibsdrは保持。
- 2026-07-22: 捕捉遅延を調査。`python/pocket_sdr.ini`の`dev_opt=-offline`はlibsdrのcase-sensitiveな`-OFFLINE`判定に一致せず、offline modeが無効。調査開始時snapshotでは`tscale=0`も通常file replayへ渡っており、cycle `ix`ごとに概ね`ix ms` sleepし、1秒分のIF消費だけで累積約499.5秒待つ状態だった（調査中に設定ファイルは`tscale=1.0`へ外部更新）。全Doppler捕捉自体も24 MHz・41 channelでは約17,220回の48,000-point FFT相当となるが、第一原因はoption case mismatchであり、`tscale=0`時は待ち時間をさらに大幅増加させる。
