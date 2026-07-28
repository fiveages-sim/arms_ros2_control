# Vendored SOEM (prebuilt)

OpenEtherCAT **SOEM v1.4.0** as `libsoem.so` only (no sources).

Required at runtime by Stanford `libhardware.so`.

```text
lib/
├── x86_64/libsoem.so
└── aarch64/libsoem.so   # add when targeting ARM
```

Built once from upstream `v1.4.0` as a shared library; see `LICENSE`.
