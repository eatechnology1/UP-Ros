<template>
  <div class="side-menu-container">
    <q-scroll-area class="menu-scroll-area" :thumb-style="{ width: '4px', opacity: '0.5' }">
      <q-list padding class="menu-list">
        <!-- =======================
             1. DASHBOARD
        ======================== -->
        <div class="menu-group q-mb-md">
          <q-item clickable v-ripple to="/" exact class="menu-item single-item">
            <q-item-section avatar>
              <q-icon name="home" class="text-blue-grey-3" />
            </q-item-section>
            <q-item-section>
              <q-item-label class="text-weight-bold">Home</q-item-label>
            </q-item-section>
          </q-item>
        </div>
        <div class="menu-group q-mb-md">
          <q-item clickable v-ripple to="/introduccion" exact class="menu-item single-item">
            <q-item-section avatar>
              <q-icon name="school" class="text-blue-grey-3" />
            </q-item-section>
            <q-item-section>
              <q-item-label class="text-weight-bold">Introducción</q-item-label>
            </q-item-section>
          </q-item>
        </div>

        <q-separator class="menu-divider" />
        <q-item-label header class="menu-header">Modulos de Aprendizaje Ros2 JAZZY</q-item-label>

        <!-- =======================
             2. MÓDULOS DINÁMICOS
        ======================== -->
        <div class="menu-group">
          <!-- Bucle principal sobre los módulos -->
          <template v-for="module in courseStructure" :key="module.path || 'unknown'">
            <!-- CORRECCIÓN: Usamos module.path || '' para evitar undefined -->
            <q-expansion-item
              v-if="module.children && module.children.length > 0"
              group="modules"
              class="module-expansion q-mb-xs"
              :header-class="[
                'module-header',
                isModuleOpen(module.path || '') ? 'module-open' : '',
              ]"
              @show="openModule = module.path || ''"
              @hide="openModule = ''"
            >
              <!-- SLOT DE CABECERA PERSONALIZADO -->
              <template v-slot:header>
                <q-item-section avatar>
                  <q-icon
                    :name="module.icon || 'folder'"
                    :class="getModuleColorClass(module.path || '')"
                    size="22px"
                  />
                </q-item-section>
                <q-item-section>
                  <q-item-label
                    :class="['text-weight-medium', getModuleColorClass(module.path || '')]"
                  >
                    {{ module.title }}
                  </q-item-label>
                </q-item-section>
              </template>

              <!-- CONTENIDO DEL MÓDULO (LECCIONES) -->
              <q-list class="topic-list-container">
                <!-- Línea guía vertical -->
                <div class="guide-line" :class="getBorderColorClass(module.path || '')"></div>

                <!-- Lista plana de lecciones -->
                <q-item
                  v-for="lesson in module.children"
                  :key="lesson.path || 'unknown-lesson'"
                  clickable
                  v-ripple
                  :to="`/${module.path}/${lesson.path}`"
                  class="menu-item sub-item"
                  active-class="active-link"
                >
                  <q-item-section>
                    <q-item-label class="sub-label">
                      {{ lesson.title }}
                    </q-item-label>
                  </q-item-section>
                </q-item>
              </q-list>
            </q-expansion-item>
          </template>
        </div>

        <q-separator class="menu-divider q-mt-lg" />
        <q-item-label header class="menu-header">HERRAMIENTAS</q-item-label>

        <!-- =======================
             3. EXTRAS
        ======================== -->
        <div class="menu-group">
          <q-item clickable v-ripple to="/glosario" class="menu-item single-item">
            <q-item-section avatar><q-icon name="menu_book" class="text-cyan-4" /></q-item-section>
            <q-item-section>Glosario</q-item-section>
          </q-item>
          <q-item clickable v-ripple to="/creditos" class="menu-item single-item">
            <q-item-section avatar><q-icon name="info" class="text-grey-5" /></q-item-section>
            <q-item-section>Créditos</q-item-section>
          </q-item>
          <q-item clickable v-ripple to="/plantilla" class="menu-item single-item">
            <q-item-section avatar><q-icon name="style" class="text-pink-4" /></q-item-section>
            <q-item-section>Plantilla Dev</q-item-section>
          </q-item>
        </div>
      </q-list>
    </q-scroll-area>

    <div class="menu-footer"><q-icon name="code" size="xs" /> UP-Ros v1.0</div>
  </div>
</template>

<script setup lang="ts">
import { ref } from 'vue';
// CAMBIO IMPORTANTE: Usamos la nueva estructura
import { courseStructure } from 'src/data/courseStructure';

// Estado para saber qué módulo está abierto y resaltarlo
const openModule = ref('');

const isModuleOpen = (path: string) => openModule.value === path;

// 1. SISTEMA DE COLORES (Actualizado para usar 'path' en lugar de 'id')
function getModuleColorClass(path: string): string {
  if (!path) return 'text-grey-4';
  if (path.includes('modulo-0')) return 'text-green-4'; // Toolset -> Green
  if (path.includes('modulo-1')) return 'text-purple-4'; // Core -> Purple
  if (path.includes('modulo-2')) return 'text-blue-4'; // Tools -> Blue
  if (path.includes('modulo-3')) return 'text-amber-4'; // Git -> Gold (Ajustado a tu estructura)
  if (path.includes('modulo-4')) return 'text-red-4'; // ROS -> Red
  if (path.includes('modulo-5')) return 'text-cyan-4'; // Tools -> Cyan
  if (path.includes('modulo-6')) return 'text-orange-4'; // Sim -> Orange
  if (path.includes('modulo-7')) return 'text-indigo-4'; // Nav -> Indigo
  if (path.includes('modulo-8')) return 'text-teal-4'; // Deploy -> Teal
  return 'text-grey-4';
}

// 2. SISTEMA DE BORDES (Para la línea guía vertical)
function getBorderColorClass(path: string): string {
  if (!path) return 'bg-grey-6';
  if (path.includes('modulo-0')) return 'bg-green-5';
  if (path.includes('modulo-1')) return 'bg-purple-5';
  if (path.includes('modulo-2')) return 'bg-blue-5';
  if (path.includes('modulo-3')) return 'bg-amber-5';
  if (path.includes('modulo-4')) return 'bg-red-5';
  if (path.includes('modulo-5')) return 'bg-cyan-5';
  if (path.includes('modulo-6')) return 'bg-orange-5';
  if (path.includes('modulo-7')) return 'bg-indigo-5';
  if (path.includes('modulo-8')) return 'bg-teal-5';
  return 'bg-grey-6';
}
</script>

<style scoped>
/* CONTAINER */
.side-menu-container {
  height: 100%;
  display: flex;
  flex-direction: column;
  background: rgba(15, 23, 42, 0.98); /* Fondo muy oscuro */
  border-right: 1px solid rgba(255, 255, 255, 0.05);
}

.menu-scroll-area {
  flex: 1;
}

/* HEADERS */
.menu-header {
  color: #475569;
  font-size: 0.7rem;
  font-weight: 800;
  letter-spacing: 0.08em;
  padding-left: 20px;
  margin-top: 12px;
}
.menu-divider {
  background: rgba(255, 255, 255, 0.05);
  margin: 16px 20px;
}

/* =========================================
   ITEMS SIMPLES (DASHBOARD / EXTRAS)
   ========================================= */
.menu-item {
  margin: 2px 12px;
  border-radius: 8px;
  color: #94a3b8;
  transition: all 0.2s ease;
  min-height: 38px;
  font-size: 0.9rem;
}

.single-item {
  margin-bottom: 4px;
}

.menu-item:hover {
  background: rgba(255, 255, 255, 0.04);
  color: #f1f5f9;
  transform: translateX(3px);
}

/* =========================================
   MÓDULOS (NIVEL 1) - LA MAGIA DEL COLOR
   ========================================= */
.module-expansion {
  border-radius: 12px;
  overflow: hidden; /* Para que el highlight no se salga */
  transition: background 0.3s;
}

/* Cuando el módulo está abierto, oscurecemos un poco el fondo */
.module-open {
  background: rgba(255, 255, 255, 0.02);
}

/* Ajuste del header del módulo */
:deep(.module-header) {
  padding: 8px 16px;
  min-height: 48px;
}

/* CONTENEDOR DE TEMAS CON LÍNEA GUÍA */
.topic-list-container {
  position: relative;
  padding-bottom: 8px;
}

/* LÍNEA VERTICAL DE COLOR */
.guide-line {
  position: absolute;
  left: 27px; /* Alineado con el icono del padre */
  top: 0;
  bottom: 10px;
  width: 2px;
  opacity: 0.3;
  border-radius: 2px;
}

/* =========================================
   SUBTEMAS (NIVEL 3 - LINKS) - AHORA NIVEL 2
   ========================================= */
.sub-item {
  margin: 1px 4px 1px 24px; /* Indentación profunda para alinearse con la línea */
  padding-left: 12px;
  border-left: 2px solid transparent; /* Preparado para borde activo */
  border-radius: 0 6px 6px 0; /* Borde solo derecha */
}

.sub-label {
  font-size: 0.8rem;
  line-height: 1.3;
}

/* ESTADO ACTIVO (SELECCIONADO) */
.active-link {
  background: linear-gradient(90deg, rgba(56, 189, 248, 0.1), transparent);
  color: #38bdf8 !important; /* Azul Sky brillante */
  border-left-color: #38bdf8; /* Borde sólido a la izquierda */
  font-weight: 600;
}

/* FOOTER */
.menu-footer {
  padding: 12px;
  text-align: center;
  font-size: 0.7rem;
  color: #475569;
  font-family: 'Fira Code', monospace;
  border-top: 1px solid rgba(255, 255, 255, 0.05);
}
</style>
