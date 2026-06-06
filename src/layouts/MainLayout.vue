<template>
  <q-layout view="hHh Lpr lFf">
    <!-- HEADER SUPERIOR (GLASS) -->
    <q-header elevated class="glass-header">
      <!-- NIVEL 1: TOOLBAR PRINCIPAL -->
      <q-toolbar class="header-toolbar">
        <!-- IZQUIERDA -->
        <q-btn
          flat
          dense
          round
          icon="menu"
          aria-label="Menu"
          class="header-icon-btn"
          @click="toggleLeftDrawer"
        />

        <!-- CENTRO -->
        <div class="toolbar-center">
          <router-link to="/" class="header-title"> UP-Ros Academy </router-link>
        </div>

        <!-- DERECHA — Botón de tema -->
        <div class="toolbar-right">
          <q-btn
            flat
            dense
            round
            :icon="currentTheme === 'dark' ? 'light_mode' : 'dark_mode'"
            :aria-label="currentTheme === 'dark' ? 'Cambiar a tema claro' : 'Cambiar a tema oscuro'"
            class="theme-toggle-btn"
            :class="{ 'theme-toggling': isToggling }"
            @click="handleToggle"
          >
            <q-tooltip anchor="bottom right" self="top right" :delay="300">
              {{ currentTheme === 'dark' ? 'Tema claro' : 'Tema oscuro' }}
            </q-tooltip>
          </q-btn>
        </div>
      </q-toolbar>

      <!-- NIVEL 2: MIGAS DE PAN (BREADCRUMBS) -->
      <!-- Solo se muestra si no estamos en el Home -->
      <div
        v-if="route.path !== '/' && route.path !== '/home'"
        class="breadcrumb-bar q-px-md q-pb-sm"
      >
        <q-breadcrumbs active-color="secondary" class="text-caption breadcrumb-text">
          <template v-slot:separator>
            <q-icon size="1.2em" name="chevron_right" class="breadcrumb-separator" />
          </template>

          <!-- Home siempre fijo -->
          <q-breadcrumbs-el icon="home" to="/" />

          <!-- Rutas Dinámicas Inteligentes -->
          <q-breadcrumbs-el
            v-for="(crumb, index) in breadcrumbs"
            :key="index"
            :label="crumb.label"
            :to="crumb.to"
            :icon="crumb.icon"
          />
        </q-breadcrumbs>
      </div>
    </q-header>

    <!-- DRAWER / MENÚ LATERAL -->
    <q-drawer v-model="leftDrawerOpen" show-if-above bordered :width="260" class="glass-drawer">
      <SideMenu />
    </q-drawer>

    <!-- CONTENIDO -->
    <q-page-container>
      <router-view />
    </q-page-container>

    <!-- FOOTER -->
    <q-footer elevated class="glass-footer">
      <div class="footer-content">
        <p class="footer-text">
          © {{ currentYear }} Alexander Calderón Leal. Todos los derechos reservados.
        </p>
      </div>
    </q-footer>
  </q-layout>
</template>

<script setup lang="ts">
import { ref, computed } from 'vue';
import { useRoute } from 'vue-router';
import SideMenu from 'components/SideMenu.vue';
import { courseStructure } from 'src/data/courseStructure';
import { useTheme } from 'src/composables/useTheme';

const leftDrawerOpen = ref(false);
const route = useRoute();
const isToggling = ref(false);

// ─── Tema ─────────────────────────────────────────
const { currentTheme, toggleTheme } = useTheme();

function handleToggle() {
  isToggling.value = true;
  toggleTheme();
  setTimeout(() => {
    isToggling.value = false;
  }, 400);
}

// ─── Año en footer ────────────────────────────────
const currentYear = computed(() => new Date().getFullYear());

// ─── Toggle drawer ────────────────────────────────
function toggleLeftDrawer() {
  leftDrawerOpen.value = !leftDrawerOpen.value;
}

// ─── Migas de pan ────────────────────────────────
const breadcrumbs = computed(() => {
  const pathArray = route.path.split('/').filter((p) => p);
  const crumbs: { label: string; to: string; icon?: string | undefined }[] = [];
  let currentPath = '';

  for (let i = 0; i < pathArray.length; i++) {
    const segment = pathArray[i];
    if (!segment) continue;

    currentPath += `/${segment}`;

    let label = segment.replace(/-/g, ' ').replace(/\b\w/g, (l) => l.toUpperCase());
    let icon: string | undefined = undefined;

    const foundModule = courseStructure.find((m) => m.path === segment);
    if (foundModule) {
      label = foundModule.title;
    } else {
      for (const mod of courseStructure) {
        if (mod.children) {
          const foundChild = mod.children.find((c) => c.path === segment);
          if (foundChild) {
            label = foundChild.title;
            icon = 'article';
            break;
          }
        }
      }
    }

    crumbs.push({ label, to: currentPath, icon });
  }

  return crumbs;
});
</script>

<style>
/* =========================
   HEADER GLASS
========================= */
.glass-header {
  background: var(--header-bg) !important;
  backdrop-filter: blur(18px) !important;
  -webkit-backdrop-filter: blur(18px) !important;
  border-bottom: 1px solid var(--border-accent) !important;
  box-shadow:
    0 4px 24px var(--shadow-sm),
    inset 0 -1px 0 var(--border-accent) !important;
  transition: background-color 0.3s ease, border-color 0.3s ease !important;
}

/* =========================
   BREADCRUMB BAR
========================= */
.breadcrumb-bar {
  display: flex;
  justify-content: center;
  border-top: 1px solid var(--border-accent);
  padding-top: 6px;
}

.breadcrumb-text {
  color: var(--text-muted) !important;
}

.breadcrumb-separator {
  color: var(--text-muted) !important;
}

.q-breadcrumbs__el {
  transition: color 0.2s;
  color: var(--text-muted) !important;
}
.q-breadcrumbs__el:hover {
  color: var(--text-code) !important;
}

/* =========================
   TÍTULO LINK
========================= */
.header-title {
  text-decoration: none;
  color: var(--text-primary);
  font-weight: 600;
  letter-spacing: 0.6px;
  transition: all 0.25s ease;
}

.header-title:hover {
  color: var(--text-code);
  text-shadow: 0 3px 10px rgba(56, 189, 248, 0.4);
}

/* =========================
   ICONOS DEL HEADER
========================= */
.header-icon-btn {
  color: var(--text-primary) !important;
}

/* =========================
   BOTÓN TOGGLE DE TEMA
========================= */
.theme-toggle-btn {
  color: var(--text-secondary) !important;
  transition: color 0.2s ease, transform 0.3s ease !important;
}

.theme-toggle-btn:hover {
  color: var(--text-code) !important;
}

/* Animación de rotación al hacer click */
.theme-toggle-btn.theme-toggling .q-icon {
  animation: spin-icon 0.4s ease-out;
}

@keyframes spin-icon {
  0%   { transform: rotate(0deg) scale(1); }
  50%  { transform: rotate(180deg) scale(1.2); }
  100% { transform: rotate(360deg) scale(1); }
}

/* =========================
   DRAWER GLASS
========================= */
.glass-drawer {
  background: var(--drawer-bg) !important;
  backdrop-filter: blur(20px) !important;
  -webkit-backdrop-filter: blur(20px) !important;
  border-right: 1px solid var(--border-accent) !important;
  transition: background-color 0.3s ease !important;
}

/* En tema claro el drawer tiene una sombra sutil */
[data-theme='light'] .glass-drawer {
  box-shadow: 2px 0 16px var(--shadow-sm) !important;
}

/* =========================
   TOOLBAR GRID
========================= */
.header-toolbar {
  display: grid;
  grid-template-columns: 48px 1fr 48px;
  align-items: center;
  min-height: 50px;
}

.toolbar-center {
  display: flex;
  justify-content: center;
}

.toolbar-right {
  display: flex;
  justify-content: flex-end;
  align-items: center;
}

/* =========================
   FOOTER GLASS
========================= */
.glass-footer {
  background: var(--footer-bg) !important;
  backdrop-filter: blur(18px) !important;
  -webkit-backdrop-filter: blur(18px) !important;
  border-top: 1px solid var(--border-accent) !important;
  box-shadow:
    0 -4px 24px var(--shadow-sm),
    inset 0 1px 0 var(--border-accent) !important;
  padding: 0.75rem 0;
  transition: background-color 0.3s ease !important;
}

.footer-content {
  max-width: 1200px;
  margin: 0 auto;
  padding: 0 1rem;
  display: flex;
  justify-content: center;
  align-items: center;
}

.footer-text {
  margin: 0;
  font-size: 0.875rem;
  color: var(--text-muted);
  text-align: center;
  letter-spacing: 0.3px;
  font-weight: 400;
}

/* RESPONSIVE */
@media (max-width: 600px) {
  .footer-text {
    font-size: 0.75rem;
    padding: 0 0.5rem;
  }
}
</style>
