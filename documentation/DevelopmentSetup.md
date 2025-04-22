# Devolopment Machine Setup

## First things first: install Git

- Git is a source control tool we use to manage and share our code
- [Windows Install](https://git-scm.com/downloads/win)
  - [Mac/Linux Instal](https://git-scm.com/book/en/v2/Getting-Started-Installing-Git)

## Then, clone the repository

1. Create a folder on your machine for all your repositories, ex: `C:/repos/`, or `C:/git/`
2. Open command prompt and navigate to the folder you want to clone your repositories in
3. Run: `git clone https://github.com/frc2357/FRC-2025.git`

## Intall NVM and Node.js

1. Node.js is used to control the formatting in our repository
2. To manage multiple version of Node, we will first install Node Version Manager

   - [Windows Install](https://github.com/coreybutler/nvm-windows/releases/latest)
   - [Linux Install](https://github.com/nvm-sh/nvm/releases/latest)

3. Once NVM is installed, open `command prompt`
4. Run `nvm install 20.18.1`
5. Run `nvm use 20.18.1`
6. Your machine is now setup to run Node 20.18.1!

## WPILib with VS Code

WPILIB is the library used to interact wih our robot's hardware

### Installing WPILIB

1. Follow this guide: [WPILIB Install Guide](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/wpilib-setup.html)
2. Once WPILib is installed, open it, then select file -> open folder and open the FRC-2025 repository that you cloned earlier

### Install Prettier Extension

1. Click on the 'extensions' button on your left-hand side bar.
2. Search for 'Prettier' and install the first option

### Setting up VSCode Settings and Formatting

1. Press `ctrl + ~` to open your console in VS Code
2. Run npm install, this will install our prettier packages that handle formatting
3. Press `crtl + lshift + p` to open the command pallete
4. Search for and select `Preferences: Open User Settings (JSON)`
5. Copy/Paste the following configs into your JSON file.

```
  "editor.defaultFormatter": "esbenp.prettier-vscode",
  "editor.formatOnSave": true,
```

6. This configures your default formatter to use prettier and formats your code when you save

### Optional Extensions

- GitLens: Gives you inline info about who changed the code your working on
- Code Spell Checker: Spell checks your code

## Install the FRC Game Tools

- The game tools include things like the Driver Station and RoboRio Imaging tool
- [Install Guide](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/frc-game-tools.html)
