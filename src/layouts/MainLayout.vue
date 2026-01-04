<template>
  <q-layout view="hHh Lpr lFf">
    <!-- HEADER SUPERIOR (GLASS) -->
    <q-header elevated class="glass-header">
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
          <router-link to="/" class="header-title"> UP-Ros </router-link>
        </div>

        <!-- DERECHA (ESPACIADOR) -->
        <div class="toolbar-spacer"></div>
      </q-toolbar>
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
import SideMenu from 'components/SideMenu.vue';

const leftDrawerOpen = ref(true);

function toggleLeftDrawer() {
  leftDrawerOpen.value = !leftDrawerOpen.value;
}

const currentYear = computed(() => new Date().getFullYear());
</script>

<style>
/* =========================
   HEADER GLASS
========================= */
.glass-header {
  background: rgba(18, 28, 45, 0.55);
  backdrop-filter: blur(18px);
  -webkit-backdrop-filter: blur(18px);

  border-bottom: 1px solid rgba(255, 255, 255, 0.08);

  box-shadow:
    0 4px 24px rgba(0, 0, 0, 0.35),
    inset 0 -1px 0 rgba(255, 255, 255, 0.04);
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
  text-shadow: 0 0 10px rgba(110, 203, 255, 0.6);
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
