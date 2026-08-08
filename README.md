# MiSTer Main – Physical Disc Support

Physical Disc Support adds direct **USB optical drive support** to MiSTer FPGA, allowing supported cores to run games from real CDs instead of disc images.

Connect a compatible USB CD/DVD drive, insert a supported disc, and launch the appropriate Physical Disc core from the MiSTer menu.

For easier setup and configuration, **MiSTer Companion** can install, update, and configure Physical Disc Support automatically.

## Supported Systems

- Sony PlayStation
- Sega Saturn
- Mega CD / Sega CD
- PC Engine CD / TurboGrafx-16 CD
- Neo Geo CD
- Philips CD-i *(experimental)*
- 3DO *(experimental / unstable)*
- SNES MSU-1
- Mega Drive / Genesis MD+

Compatibility can vary depending on the game, core, and USB optical drive being used.

## Requirements

- MiSTer FPGA
- Compatible USB CD/DVD drive
- Physical Disc version of MiSTer Main
- Physical Disc compatible cores

## Installation

### MiSTer Companion

The easiest way to install and configure Physical Disc Support is with **MiSTer Companion**.

Download:

https://mistercompanion.org

Open the **Install Center** and install **Physical Disc Cores**.

MiSTer Companion can:

- Install the required Physical Disc files
- Update Physical Disc Support
- Configure the required `MiSTer.ini` entries
- Enable or disable Auto Disc Detection

This is the recommended installation method for most users.

### Update_All

Physical Disc Support can also be installed through **Update_All** using its Downloader database.

Once enabled, running Update_All will install and update the required Physical Disc files.

## Usage

### Manual Launch

Manual launching is the standard way to use Physical Disc Support and does not require Auto Disc Detection.

Physical Disc Support installs a **`_Physical Disc Cores`** folder in the MiSTer menu.

This folder contains MGL files for the supported Physical Disc cores.

To launch a physical disc:

1. Connect your USB optical drive.
2. Insert a supported disc.
3. Open **`_Physical Disc Cores`** from the MiSTer menu.
4. Select the MGL for the system you want to use.
5. The corresponding Physical Disc core will start and use the inserted disc.

The MGL files make it easy to manually select which Physical Disc core should be used.

They are also useful for Audio CDs, as you can simply launch the MGL for the core you want to use.

## Auto Disc Detection

Auto Disc Detection is an optional extra feature that can automatically identify a newly inserted disc and launch the appropriate Physical Disc core.

When enabled:

1. Start MiSTer normally.
2. Insert a supported disc.
3. Physical Disc Support identifies the disc.
4. The appropriate core is launched automatically.

A disc that is already inserted when MiSTer starts is not automatically launched. Auto Disc Detection reacts to a disc being inserted after MiSTer has started.

MiSTer Companion provides an easy option to enable or disable Auto Disc Detection without manually editing `MiSTer.ini`.

## MiSTer.ini Configuration

Physical Disc Support uses dedicated `MiSTer.ini` sections following this pattern:

```ini
[A0CD-*]
main=MiSTer_Physical-CD
```

The `*` represents the Physical Disc profile being used.

MiSTer Companion can configure the required entries automatically.

### Enabling Auto Disc Detection

To enable Auto Disc Detection manually, add the following to your existing `[menu]` section:

```ini
main=MiSTer_Physical-CD
```

For example:

```ini
[menu]
main=MiSTer_Physical-CD
```

## Audio CD Override

When Auto Disc Detection is enabled, standard Audio CDs use the **PlayStation profile by default**.

If you prefer Audio CDs to launch using another supported Physical Disc profile, add an `AUDIOCD=` override to your existing `[menu]` section.

For example, to use the Sega Saturn profile:

```ini
AUDIOCD=A0CD-Saturn
```

Supported Audio CD overrides:

```ini
AUDIOCD=A0CD-PSX
AUDIOCD=A0CD-Saturn
AUDIOCD=A0CD-MegaCD
AUDIOCD=A0CD-TurboGrafx16-CD
AUDIOCD=A0CD-NeoGeoCD
AUDIOCD=A0CD-CDi
AUDIOCD=A0CD-3DO
```

Only use **one** `AUDIOCD=` entry.

For example:

```ini
[menu]
main=MiSTer_Physical-CD
AUDIOCD=A0CD-Saturn
```

The Audio CD override is only relevant when using Auto Disc Detection.

When Auto Disc Detection is disabled, simply open **`_Physical Disc Cores`** and launch the MGL for the core you want to use.

## Disc Swapping

Supported cores can detect physical disc removal and insertion while running.

This allows Physical Disc Support to handle things such as:

- Multi-disc games
- Games that request a disc change
- Games that allow the game disc to be replaced while the core remains running

Support depends on the individual core and game.

## CD Audio

Physical Disc Support supports games that use CD audio, including mixed-mode discs containing both game data and Red Book audio tracks.

Because the audio is being read from a real optical drive rather than a disc image, compatibility and timing can vary between drives and games.

## Burning SNES MSU-1 & Mega Drive / Genesis MD+ CDs

For users who want to create physical discs for **SNES MSU-1** or **Mega Drive / Genesis MD+**, a dedicated burning tutorial is included with the project.

The tutorial was written by **NinoPrime** and covers the process of preparing and burning compatible discs for use with Physical Disc Support.

> **Note:** The tutorial currently covers the process on **Windows only**.

[Burning MSU-1 & MD+ CDs](assets/Burning%20MSU-1%20%26%20MD%2B%20CD%27s.docx)

## Updating

Physical Disc Support can be updated through:

- MiSTer Companion
- Update_All

MiSTer Companion:

https://mistercompanion.org

## Known Issues

Physical Disc Support is still under active development.

Current known issues include:

- On PlayStation, some mixed-mode games such as **Ridge Racer** currently only play the first CD audio track correctly. Audio tracks beyond track 1 may not play.
- **Audio CD swapping on PlayStation is currently not working.**
- Some games may still require additional timing or CD audio compatibility fixes.
- USB optical drive compatibility can vary between models.
- Philips CD-i support is experimental.
- 3DO support is currently unstable.

## Reporting Issues

When reporting an issue, please include:

- System/core
- Game title
- Region
- USB optical drive model
- Whether the issue also happens when using a disc image
- Whether the problem affects loading, CD audio, disc detection, or disc swapping

## Credits

- **NinoPrime** — author of the SNES MSU-1 & Mega Drive / Genesis MD+ CD burning tutorial

This project builds on MiSTer Main, the supported MiSTer cores, and the work of the MiSTer FPGA community.

MiSTer FPGA:

https://github.com/MiSTer-devel
