# Applications

This firmware can be built for different hardware targets, depending on the selected `APP`.

Currently supported:
- [Default](./default/)
- [Blus mini](./blus_mini/)

From the firmware root directory, build with

```bash
make APP=<app_name>
```

where `<app_name>` is the name of the subdirectory within `apps` that contains the app to build.
