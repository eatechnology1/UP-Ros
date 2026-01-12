<template>
  <div class="side-menu-container">
    <!-- BUSCADOR RÁPIDO -->
    <div class="search-bar q-pa-md">
      <q-input
        v-model="searchQuery"
        dense
        standout="bg-grey-9"
        placeholder="Buscar lecciones..."
        class="search-input"
        @update:model-value="handleSearch"
      >
        <template v-slot:prepend>
          <q-icon name="search" class="text-grey-5" />
        </template>
        <template v-slot:append v-if="searchQuery">
          <q-icon name="close" class="cursor-pointer text-grey-5" @click="clearSearch" />
        </template>
      </q-input>
    </div>

    <q-scroll-area class="menu-scroll-area" :thumb-style="{ width: '4px', opacity: '0.5' }">
      <q-list padding class="menu-list">
        <!-- HOME -->
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

        <div class="menu-group q-mb-md">
          <q-item clickable v-ripple to="/instalacion" exact class="menu-item single-item">
            <q-item-section avatar>
              <q-icon name="download" class="text-blue-grey-3" />
            </q-item-section>
            <q-item-section>
              <q-item-label class="text-weight-bold">Instalación</q-item-label>
            </q-item-section>
          </q-item>
        </div>

        <q-separator class="menu-divider" />
        <q-item-label header class="menu-header">Módulos de Aprendizaje ROS2 JAZZY</q-item-label>

        <!-- MÓDULOS DINÁMICOS CON TOOLTIPS -->
        <div class="menu-group">
          <template v-for="module in filteredModules" :key="module.path || 'unknown'">
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
              <!-- HEADER CON TOOLTIP -->
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
                  <!-- INDICADORES DE METADATA -->
                  <q-item-label caption class="module-meta q-mt-xs">
                    <span
                      v-if="module.difficulty"
                      class="difficulty-badge"
                      :class="`difficulty-${module.difficulty}`"
                    >
                      {{ getDifficultyLabel(module.difficulty) }}
                    </span>
                    <span v-if="module.estimatedTime" class="time-badge">
                      <q-icon name="schedule" size="12px" /> {{ module.estimatedTime }}
                    </span>
                  </q-item-label>
                </q-item-section>

                <!-- TOOLTIP TÉCNICO -->
                <q-tooltip
                  v-if="module.tooltip"
                  anchor="center right"
                  self="center left"
                  :offset="[10, 0]"
                  class="technical-tooltip"
                  transition-show="scale"
                  transition-hide="scale"
                >
                  <div class="tooltip-content">
                    <div class="tooltip-title">{{ module.title }}</div>
                    <div class="tooltip-description">{{ module.tooltip }}</div>
                    <div v-if="module.tags && module.tags.length > 0" class="tooltip-tags q-mt-sm">
                      <q-chip
                        v-for="tag in module.tags"
                        :key="tag"
                        dense
                        size="sm"
                        class="tag-chip"
                      >
                        {{ tag }}
                      </q-chip>
                    </div>
                  </div>
                </q-tooltip>
              </template>

              <!-- LECCIONES -->
              <q-list class="topic-list-container">
                <div class="guide-line" :class="getBorderColorClass(module.path || '')"></div>

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

        <!-- EXTRAS -->
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
import { ref, computed } from 'vue';
import { courseStructure, type CourseNode } from 'src/data/courseStructure';

const openModule = ref('');
const searchQuery = ref('');

const isModuleOpen = (path: string) => openModule.value === path;

// BÚSQUEDA INTELIGENTE
const filteredModules = computed(() => {
  if (!searchQuery.value.trim()) {
    return courseStructure;
  }

  const query = searchQuery.value.toLowerCase();

  return courseStructure
    .map((module) => {
      // Buscar en el título del módulo
      const moduleMatches = module.title.toLowerCase().includes(query);

      // Buscar en las lecciones hijas
      const matchingChildren = module.children?.filter(
        (lesson) =>
          lesson.title.toLowerCase().includes(query) ||
          lesson.description?.toLowerCase().includes(query),
      );

      // Si el módulo coincide, devolver todo
      if (moduleMatches) {
        return module;
      }

      // Si hay lecciones que coinciden, devolver módulo con solo esas lecciones
      if (matchingChildren && matchingChildren.length > 0) {
        return { ...module, children: matchingChildren };
      }

      return null;
    })
    .filter((module): module is CourseNode => module !== null);
});

function handleSearch() {
  // Auto-expandir el primer módulo con resultados
  if (filteredModules.value.length > 0 && searchQuery.value.trim()) {
    openModule.value = filteredModules.value[0]?.path || '';
  }
}

function clearSearch() {
  searchQuery.value = '';
  openModule.value = '';
}

// SISTEMA DE COLORES
function getModuleColorClass(path: string): string {
  if (!path) return 'text-grey-4';
  if (path.includes('modulo-0')) return 'text-green-4';
  if (path.includes('modulo-1')) return 'text-purple-4';
  if (path.includes('modulo-2')) return 'text-blue-4';
  if (path.includes('modulo-3')) return 'text-amber-4';
  if (path.includes('modulo-4')) return 'text-red-4';
  if (path.includes('modulo-5')) return 'text-cyan-4';
  if (path.includes('modulo-6')) return 'text-orange-4';
  if (path.includes('modulo-7')) return 'text-indigo-4';
  if (path.includes('modulo-8')) return 'text-teal-4';
  return 'text-grey-4';
}

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

function getDifficultyLabel(difficulty: string): string {
  const labels: Record<string, string> = {
    beginner: 'Básico',
    intermediate: 'Intermedio',
    advanced: 'Avanzado',
  };
  return labels[difficulty] || difficulty;
}
</script>

<style scoped>
/* ========================================
   CONTAINER
   ======================================== */
.side-menu-container {
  height: 100%;
  display: flex;
  flex-direction: column;
  background: rgba(15, 23, 42, 0.98);
  border-right: 1px solid rgba(255, 255, 255, 0.05);
}

/* ========================================
   BUSCADOR RÁPIDO
   ======================================== */
.search-bar {
  border-bottom: 1px solid rgba(255, 255, 255, 0.05);
  background: rgba(0, 0, 0, 0.2);
}

.search-input :deep(.q-field__control) {
  border-radius: 8px;
  background: rgba(255, 255, 255, 0.03);
  transition: all 0.3s ease;
}

.search-input :deep(.q-field__control):hover {
  background: rgba(255, 255, 255, 0.06);
}

.search-input :deep(.q-field__control):focus-within {
  background: rgba(255, 255, 255, 0.08);
  box-shadow: 0 0 0 2px rgba(56, 189, 248, 0.3);
}

.menu-scroll-area {
  flex: 1;
}

/* ========================================
   HEADERS Y DIVIDERS
   ======================================== */
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

/* ========================================
   ITEMS SIMPLES
   ======================================== */
.menu-item {
  margin: 2px 12px;
  border-radius: 8px;
  color: #94a3b8;
  transition: all 0.25s cubic-bezier(0.4, 0, 0.2, 1);
  min-height: 38px;
  font-size: 0.9rem;
}

.single-item {
  margin-bottom: 4px;
}

.menu-item:hover {
  background: rgba(255, 255, 255, 0.06);
  color: #f1f5f9;
  transform: translateX(4px);
  box-shadow: 0 2px 8px rgba(0, 0, 0, 0.2);
}

/* ========================================
   MÓDULOS (EXPANSIÓN)
   ======================================== */
.module-expansion {
  border-radius: 12px;
  overflow: hidden;
  transition: all 0.3s ease;
  margin: 4px 8px;
}

.module-expansion:hover {
  background: rgba(255, 255, 255, 0.03);
}

.module-open {
  background: rgba(255, 255, 255, 0.04) !important;
}

:deep(.module-header) {
  padding: 10px 16px;
  min-height: 52px;
  transition: all 0.2s ease;
}

:deep(.module-header):hover {
  background: rgba(255, 255, 255, 0.02);
}

/* ========================================
   METADATA BADGES
   ======================================== */
.module-meta {
  display: flex;
  gap: 8px;
  align-items: center;
  margin-top: 4px;
}

.difficulty-badge {
  font-size: 0.65rem;
  font-weight: 700;
  text-transform: uppercase;
  letter-spacing: 0.05em;
  padding: 2px 8px;
  border-radius: 4px;
  background: rgba(255, 255, 255, 0.1);
}

.difficulty-beginner {
  color: #4ade80;
  background: rgba(74, 222, 128, 0.15);
}

.difficulty-intermediate {
  color: #fbbf24;
  background: rgba(251, 191, 36, 0.15);
}

.difficulty-advanced {
  color: #f87171;
  background: rgba(248, 113, 113, 0.15);
}

.time-badge {
  font-size: 0.65rem;
  color: #94a3b8;
  display: flex;
  align-items: center;
  gap: 4px;
}

/* ========================================
   TOOLTIPS TÉCNICOS
   ======================================== */
.technical-tooltip {
  background: linear-gradient(135deg, rgba(15, 23, 42, 0.98), rgba(30, 41, 59, 0.98));
  backdrop-filter: blur(12px);
  border: 1px solid rgba(56, 189, 248, 0.3);
  border-radius: 12px;
  padding: 16px;
  max-width: 320px;
  box-shadow: 0 8px 32px rgba(0, 0, 0, 0.4);
}

.tooltip-content {
  color: #e2e8f0;
}

.tooltip-title {
  font-weight: 700;
  font-size: 0.9rem;
  color: #38bdf8;
  margin-bottom: 8px;
  letter-spacing: 0.02em;
}

.tooltip-description {
  font-size: 0.8rem;
  line-height: 1.5;
  color: #cbd5e1;
}

.tooltip-tags {
  display: flex;
  flex-wrap: wrap;
  gap: 6px;
}

.tag-chip {
  background: rgba(56, 189, 248, 0.15);
  color: #38bdf8;
  font-size: 0.65rem;
  font-weight: 600;
  border: 1px solid rgba(56, 189, 248, 0.3);
}

/* ========================================
   LECCIONES (HIJOS)
   ======================================== */
.topic-list-container {
  position: relative;
  padding-bottom: 8px;
}

.guide-line {
  position: absolute;
  left: 27px;
  top: 0;
  bottom: 10px;
  width: 2px;
  opacity: 0.3;
  border-radius: 2px;
}

.sub-item {
  margin: 1px 4px 1px 24px;
  padding-left: 12px;
  border-left: 2px solid transparent;
  border-radius: 0 6px 6px 0;
  transition: all 0.2s ease;
}

.sub-item:hover {
  background: rgba(255, 255, 255, 0.04);
  transform: translateX(2px);
}

.sub-label {
  font-size: 0.8rem;
  line-height: 1.4;
}

.active-link {
  background: linear-gradient(90deg, rgba(56, 189, 248, 0.15), transparent);
  color: #38bdf8 !important;
  border-left-color: #38bdf8;
  font-weight: 600;
  box-shadow: 0 2px 8px rgba(56, 189, 248, 0.2);
}

/* ========================================
   FOOTER
   ======================================== */
.menu-footer {
  padding: 12px;
  text-align: center;
  font-size: 0.7rem;
  color: #475569;
  font-family: 'Fira Code', monospace;
  border-top: 1px solid rgba(255, 255, 255, 0.05);
  background: rgba(0, 0, 0, 0.2);
}

/* ========================================
   ANIMACIONES
   ======================================== */
@keyframes slideIn {
  from {
    opacity: 0;
    transform: translateX(-10px);
  }
  to {
    opacity: 1;
    transform: translateX(0);
  }
}

.menu-item,
.module-expansion {
  animation: slideIn 0.3s ease-out;
}
</style>
