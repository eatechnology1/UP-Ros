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
          class="text-white"
          @click="toggleLeftDrawer"
        />

        <!-- CENTRO -->
        <div class="toolbar-center">
          <router-link to="/" class="header-title"> UP-Ros Academy </router-link>
        </div>

        <!-- DERECHA (ESPACIADOR) -->
        <div class="toolbar-spacer"></div>
      </q-toolbar>

      <!-- NIVEL 2: MIGAS DE PAN (BREADCRUMBS) -->
      <!-- Solo se muestra si no estamos en el Home -->
      <div
        v-if="route.path !== '/' && route.path !== '/home'"
        class="breadcrumb-bar q-px-md q-pb-sm"
      >
        <q-breadcrumbs active-color="secondary" class="text-grey-5 text-caption">
          <template v-slot:separator>
            <q-icon size="1.2em" name="chevron_right" color="grey-6" />
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
import { courseStructure } from 'src/data/courseStructure'; // Importamos la estructura

const leftDrawerOpen = ref(true);
const route = useRoute();

function toggleLeftDrawer() {
  leftDrawerOpen.value = !leftDrawerOpen.value;
}

const currentYear = computed(() => new Date().getFullYear());

// --- LÓGICA DE MIGAS DE PAN MEJORADA ---
const breadcrumbs = computed(() => {
  const pathArray = route.path.split('/').filter((p) => p);

  // Array acumulador para las migas
  const crumbs = [];
  let currentPath = '';

  // Recorremos cada segmento de la URL
  for (let i = 0; i < pathArray.length; i++) {
    const segment = pathArray[i];
    if (!segment) continue;

    currentPath += `/${segment}`;

    // Buscamos el título bonito en courseStructure
    let label = segment.replace(/-/g, ' ').replace(/\b\w/g, (l) => l.toUpperCase());
    let icon = undefined;

    // Lógica de búsqueda en el árbol JSON
    // Nivel 1: Módulos (ej: modulo-0)
    const foundModule = courseStructure.find((m) => m.path === segment);

    if (foundModule) {
      label = foundModule.title; // "Módulo 0: Fundamentos"
      // No ponemos icono en módulos intermedios para no saturar
    } else {
      // Nivel 2: Lecciones (ej: nav-sistema)
      // Buscamos dentro de todos los módulos a ver si algún hijo coincide
      for (const mod of courseStructure) {
        if (mod.children) {
          const foundChild = mod.children.find((c) => c.path === segment);
          if (foundChild) {
            label = foundChild.title; // "0.1 Navegación Terminal"
            icon = 'article';
            break;
          }
        }
      }
    }

    crumbs.push({
      label: label,
      to: currentPath,
      icon: icon,
    });
  }

  return crumbs;
});
</script>

<style>
/* =========================
   HEADER GLASS
========================= */
.glass-header {
  background: rgba(18, 28, 45, 0.75); /* Un poco más oscuro para legibilidad */
  backdrop-filter: blur(18px);
  -webkit-backdrop-filter: blur(18px);

  border-bottom: 1px solid rgba(255, 255, 255, 0.08);

  box-shadow:
    0 4px 24px rgba(0, 0, 0, 0.35),
    inset 0 -1px 0 rgba(255, 255, 255, 0.04);

  /* Transición suave de altura */
  transition: height 0.3s ease;
}

/* =========================
   BREADCRUMB BAR
========================= */
.breadcrumb-bar {
  display: flex;
  justify-content: center; /* Centrado estético */
  border-top: 1px solid rgba(255, 255, 255, 0.03);
  padding-top: 6px;
}

/* Ajuste fino de los enlaces del breadcrumb */
.q-breadcrumbs__el {
  transition: color 0.2s;
}
.q-breadcrumbs__el:hover {
  color: #6ecbff !important; /* Azul cyan al pasar el mouse */
}

/* =========================
   TÍTULO LINK
========================= */
.header-title {
  text-decoration: none;
  color: #e6edf7;
  font-weight: 600;
  letter-spacing: 0.6px;
  transition: all 0.25s ease;
}

.header-title:hover {
  color: #6ecbff;
  text-shadow: 0 3px 10px rgba(110, 203, 255, 0.6);
}

/* =========================
   DRAWER GLASS (COHERENCIA)
========================= */
.glass-drawer {
  background: rgba(15, 25, 40, 0.6);
  backdrop-filter: blur(20px);
  -webkit-backdrop-filter: blur(20px);
  border-right: 1px solid rgba(255, 255, 255, 0.08);
}

/* =========================
   TOOLBAR GRID
========================= */
.header-toolbar {
  display: grid;
  grid-template-columns: 48px 1fr 48px;
  align-items: center;
  min-height: 50px; /* Altura controlada */
}

/* =========================
   CENTRO REAL
========================= */
.toolbar-center {
  display: flex;
  justify-content: center;
}

/* =========================
   ESPACIADOR DERECHO
========================= */
.toolbar-spacer {
  width: 48px;
}

/* =========================
   FOOTER GLASS (COHERENTE)
========================= */
.glass-footer {
  background: rgba(15, 25, 40, 0.55);
  backdrop-filter: blur(18px);
  -webkit-backdrop-filter: blur(18px);
  border-top: 1px solid rgba(255, 255, 255, 0.08);
  box-shadow:
    0 -4px 24px rgba(0, 0, 0, 0.35),
    inset 0 1px 0 rgba(255, 255, 255, 0.04);
  padding: 0.75rem 0;
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
  color: #9ca3af;
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
