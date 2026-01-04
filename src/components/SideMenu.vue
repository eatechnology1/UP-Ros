<template>
  <div class="side-menu">
    <q-list padding>
      <q-item
        v-for="item in menu"
        :key="item.id"
        clickable
        v-ripple
        class="menu-item"
        :to="item.route"
        exact
      >
        <q-item-section avatar>
          <q-icon :name="item.icon" />
        </q-item-section>

        <q-item-section>
          <q-item-label>{{ item.label }}</q-item-label>
        </q-item-section>
      </q-item>
    </q-list>
  </div>
</template>

<script setup lang="ts">
const menu = [
  { id: 0, label: 'Home', icon: 'home', route: '/' },
  { id: 1, label: 'Introducción', icon: 'circle', route: '/introduccion' },
  { id: 2, label: 'Instalación', icon: 'download', route: '/instalacion' },
  { id: 3, label: 'Fundamentos', icon: 'info', route: '/fundamentos' },
  { id: 4, label: 'Ejemplos prácticos', icon: 'code', route: '/ejemplos' },
  { id: 5, label: 'Simulación', icon: 'inventory_2', route: '/simulacion' },
  { id: 6, label: 'Proyecto con Robot', icon: 'smart_toy', route: '/proyecto' },
  { id: 7, label: 'Glosario', icon: 'menu_book', route: '/glosario' },
  { id: 8, label: 'Créditos', icon: 'school', route: '/creditos' },
];
</script>

<style scoped>
/* CONTENEDOR GENERAL */
.side-menu {
  height: 100%;
  padding: 16px;

  background: rgba(20, 30, 50, 0.55);
  backdrop-filter: blur(18px);
  -webkit-backdrop-filter: blur(18px);

  border-right: 1px solid rgba(255, 255, 255, 0.08);
  box-shadow: inset -1px 0 0 rgba(255, 255, 255, 0.04);
}

/* ITEM GLASS */
.menu-item {
  margin-bottom: 14px;
  padding: 10px 6px;

  border-radius: 16px;

  background: linear-gradient(135deg, rgba(255, 255, 255, 0.08), rgba(255, 255, 255, 0.02));

  backdrop-filter: blur(12px);
  -webkit-backdrop-filter: blur(12px);

  border: 1px solid rgba(255, 255, 255, 0.1);

  transition: all 0.25s ease;
}

/* TEXTO */
.menu-item .q-item__label {
  font-weight: 500;
  letter-spacing: 0.2px;
  color: #e6edf7;
}

/* ICONO */
.menu-item .q-icon {
  color: #6ecbff;
}

/* HOVER */
.menu-item:hover {
  transform: translateX(6px);
  background: linear-gradient(135deg, rgba(110, 203, 255, 0.25), rgba(110, 203, 255, 0.05));

  box-shadow:
    0 0 12px rgba(110, 203, 255, 0.35),
    inset 0 0 12px rgba(255, 255, 255, 0.06);
}

/* ACTIVO */
.menu-item.router-link-active,
.menu-item.router-link-exact-active {
  background: linear-gradient(135deg, rgba(110, 203, 255, 0.35), rgba(110, 203, 255, 0.12));

  box-shadow:
    0 0 18px rgba(110, 203, 255, 0.45),
    inset 0 0 16px rgba(255, 255, 255, 0.08);

  border-color: rgba(110, 203, 255, 0.45);
}
</style>
