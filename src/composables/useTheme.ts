/**
 * useTheme — Composable para gestión global del tema claro/oscuro
 * Persiste la preferencia en localStorage.
 * Integra con Quasar Dark plugin y la clase [data-theme] del <html>.
 */

import { ref, watch } from 'vue';
import { useQuasar } from 'quasar';

export type ThemeMode = 'dark' | 'light';

const STORAGE_KEY = 'upros-theme';
const DEFAULT_THEME: ThemeMode = 'dark';

// Estado global compartido entre todos los componentes
const currentTheme = ref<ThemeMode>(
  (localStorage.getItem(STORAGE_KEY) as ThemeMode) ?? DEFAULT_THEME,
);

export function useTheme() {
  const $q = useQuasar();

  /**
   * Aplica el tema al DOM y a Quasar
   */
  function applyTheme(theme: ThemeMode) {
    // 1. Actualizar atributo en <html> para los selectores CSS
    document.documentElement.setAttribute('data-theme', theme);

    // 2. Sincronizar con el sistema de dark mode de Quasar
    $q.dark.set(theme === 'dark');

    // 3. Persistir
    localStorage.setItem(STORAGE_KEY, theme);
  }

  /**
   * Alterna entre dark y light
   */
  function toggleTheme() {
    const next: ThemeMode = currentTheme.value === 'dark' ? 'light' : 'dark';
    currentTheme.value = next;
  }

  /**
   * Establece un tema específico
   */
  function setTheme(theme: ThemeMode) {
    currentTheme.value = theme;
  }

  // Reaccionar a cambios en el estado
  watch(
    currentTheme,
    (theme) => {
      applyTheme(theme);
    },
    { immediate: true },
  );

  return {
    currentTheme,
    isDark: () => currentTheme.value === 'dark',
    isLight: () => currentTheme.value === 'light',
    toggleTheme,
    setTheme,
    applyTheme,
  };
}
