import { app, BrowserWindow } from 'electron';
import path from 'path';
import os from 'os';
import { fileURLToPath } from 'url';

const platform = process.platform || os.platform();
const currentDir = fileURLToPath(new URL('.', import.meta.url));

let mainWindow: BrowserWindow | undefined;

async function createWindow() {
  mainWindow = new BrowserWindow({
    icon: path.resolve(currentDir, 'icons/icon.png'),
    width: 1000,
    height: 600,
    useContentSize: true,
    show: false, // ⬅️ SIEMPRE false
    webPreferences: {
      contextIsolation: true,
      nodeIntegration: false,
      // ⚠️ CRÍTICO: Deshabilitar webSecurity para permitir iframes de YouTube
      // Esta es la ÚNICA solución que funciona para Error 153 en Electron
      webSecurity: false,
      preload: path.resolve(
        currentDir,
        path.join(
          process.env.QUASAR_ELECTRON_PRELOAD_FOLDER,
          'electron-preload' + process.env.QUASAR_ELECTRON_PRELOAD_EXTENSION,
        ),
      ),
    },
  });

  // Cargar contenido
  if (process.env.DEV) {
    await mainWindow.loadURL(process.env.APP_URL);
  } else {
    await mainWindow.loadFile('index.html');
  }

  // 🔑 FORZAR MAXIMIZE ANTES DE MOSTRAR
  mainWindow.maximize();

  // 🔑 Mostrar solo cuando el contenido ya existe
  mainWindow.show();

  // Debug
  if (process.env.DEBUGGING) {
    // mainWindow.webContents.openDevTools();
  } else {
    mainWindow.webContents.on('devtools-opened', () => {
      mainWindow?.webContents.closeDevTools();
    });
  }

  mainWindow.on('closed', () => {
    mainWindow = undefined;
  });
}

// CORRECCIÓN 1: Usar void
void app.whenReady().then(createWindow);

app.on('window-all-closed', () => {
  if (platform !== 'darwin') {
    app.quit();
  }
});

app.on('activate', () => {
  if (!mainWindow) {
    // CORRECCIÓN 2: Usar void
    void createWindow();
  }
});
