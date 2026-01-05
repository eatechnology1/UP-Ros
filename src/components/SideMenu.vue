<template>
  <div class="side-menu-container">
    <q-list padding class="menu-list">
      <!-- SECCIÓN: INICIO -->
      <div class="menu-group">
        <q-item clickable v-ripple to="/" exact class="menu-item">
          <q-item-section avatar>
            <q-icon name="dashboard" />
          </q-item-section>
          <q-item-section>
            <q-item-label>Dashboard</q-item-label>
          </q-item-section>
        </q-item>
      </div>

      <q-separator class="menu-divider" />
      <q-item-label header class="menu-header">Ruta de Aprendizaje</q-item-label>

      <!-- SECCIÓN: MÓDULOS -->
      <div class="menu-group">
        <q-item
          v-for="item in learningModules"
          :key="item.path"
          clickable
          v-ripple
          :to="item.path"
          class="menu-item"
        >
          <q-item-section avatar>
            <q-icon :name="item.icon" />
          </q-item-section>
          <q-item-section>
            <q-item-label>{{ item.label }}</q-item-label>
          </q-item-section>
        </q-item>
      </div>

      <q-separator class="menu-divider" />
      <q-item-label header class="menu-header">Recursos Extra</q-item-label>

      <!-- SECCIÓN: EXTRAS -->
      <div class="menu-group">
        <q-item
          v-for="item in extraResources"
          :key="item.path"
          clickable
          v-ripple
          :to="item.path"
          class="menu-item"
        >
          <q-item-section avatar>
            <q-icon :name="item.icon" />
          </q-item-section>
          <q-item-section>
            <q-item-label>{{ item.label }}</q-item-label>
          </q-item-section>
        </q-item>
      </div>
    </q-list>

    <!-- Footer pequeño de versión (Toque profesional) -->
    <div class="menu-footer">UpROS v1.0.0</div>
  </div>
</template>

<script setup lang="ts">
// Separamos la data para tener mejor control
const learningModules = [
  { label: 'Introducción', icon: 'flag', path: '/introduccion' },
  { label: 'Fundamentos', icon: 'layers', path: '/fundamentos' }, // Icono más abstracto
  { label: 'Instalación', icon: 'terminal', path: '/instalacion' }, // Terminal es más tech
  { label: 'Ejemplos Prácticos', icon: 'code', path: '/ejemplos' },
  { label: 'Simulación', icon: 'view_in_ar', path: '/simulacion' }, // AR/3D view
  { label: 'Proyecto Robot', icon: 'smart_toy', path: '/proyecto' }, // Icono de robot
];

const extraResources = [
  { label: 'Glosario', icon: 'menu_book', path: '/glosario' },
  { label: 'Créditos', icon: 'info', path: '/creditos' },
];
</script>

<style scoped>
/* CONTENEDOR PRINCIPAL */
.side-menu-container {
  height: 100%;
  display: flex;
  flex-direction: column;
  background: rgba(15, 23, 42, 0.95); /* Slate-900 sólido con toque de transp */
  backdrop-filter: blur(10px);
  border-right: 1px solid rgba(148, 163, 184, 0.1);
}

.menu-list {
  flex: 1; /* Ocupa el espacio disponible */
}

/* HEADERS Y SEPARADORES */
.menu-header {
  color: #64748b; /* Slate-500 */
  font-size: 0.75rem;
  font-weight: 700;
  text-transform: uppercase;
  letter-spacing: 0.05em;
  padding-left: 24px;
  margin-top: 8px;
}

.menu-divider {
  background: rgba(148, 163, 184, 0.1);
  margin: 12px 16px;
}

/* ESTILOS DE ITEM (BASE) */
.menu-item {
  margin: 4px 12px; /* Margen lateral para que flote un poco */
  border-radius: 8px;
  color: #94a3b8; /* Slate-400 (Inactivo) */
  transition: all 0.3s cubic-bezier(0.4, 0, 0.2, 1);
  font-weight: 500;
}

.menu-item .q-icon {
  font-size: 20px;
  transition: color 0.3s;
}

/* HOVER (Sutil) */
.menu-item:hover {
  background: rgba(255, 255, 255, 0.03);
  color: #e2e8f0; /* Slate-200 */
  transform: translateX(4px); /* Pequeño movimiento */
}

.menu-item:hover .q-icon {
  color: #38bdf8; /* Sky-400 */
}

/* ACTIVO (El estado importante) */
.menu-item.q-router-link--active {
  background: rgba(56, 189, 248, 0.1); /* Sky-400 muy suave fondo */
  color: #38bdf8; /* Sky-400 Texto */
  font-weight: 600;
  position: relative;
}

/* Barra lateral brillante para el activo */
.menu-item.q-router-link--active::before {
  content: '';
  position: absolute;
  left: -8px; /* Fuera del item */
  top: 10%;
  bottom: 10%;
  width: 3px;
  border-radius: 2px;
  background: #38bdf8;
  box-shadow: 0 0 8px rgba(56, 189, 248, 0.6);
}

.menu-item.q-router-link--active .q-icon {
  color: #38bdf8;
}

/* FOOTER */
.menu-footer {
  padding: 16px;
  text-align: center;
  font-size: 0.7rem;
  color: #475569; /* Slate-600 */
  font-family: 'Fira Code', monospace;
  opacity: 0.6;
}
</style>
