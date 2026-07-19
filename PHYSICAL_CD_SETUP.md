# MiSTer Physical CD main — v0.6 PSX base

This source preserves the proven v0.6 PSX physical-disc reader while changing
the custom launch naming to the generic `CD-…` convention.

## Build name

Build normally and rename the resulting binary:

```sh
make clean
make
mv bin/MiSTer bin/MiSTer_Physical-CD
```

Install it at:

```text
/media/fat/MiSTer_Physical-CD
```

## MiSTer.ini

One wildcard section routes every future physical-CD setname to the same main:

```ini
[CD-*]
main=MiSTer_Physical-CD
```

MiSTer's INI parser treats `*` as a prefix wildcard, so this matches `CD-PSX`,
`CD-Saturn`, and other future adapters.

## PSX physical-disc MGL

```xml
<mistergamedescription>
    <rbf>_Custom Cores/Cores/PSX</rbf>
    <setname same_dir="1">CD-PSX</setname>
</mistergamedescription>
```

The modified PSX RBF containing `Use Physical Disc` remains unchanged. This
main recognizes `CD-PSX`, the older `PSX-CD` name, and the regular internal
`PSX` name for compatibility.

## Current adapter status

- `CD-PSX`: supported using the v0.6 reader.
- Other `CD-…` setnames: routed to this binary by the wildcard but require
  their corresponding core adapter before physical discs will work.

Keep Zaparoo's optical-disc reader disabled during physical-disc playback so it
does not issue competing filesystem reads against `/dev/sr0`.
