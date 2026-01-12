<template>
  <q-page class="module-landing-page q-pa-lg">
    <div v-if="currentModule" class="content-container">
      <!-- Header del Módulo -->
      <div class="module-header q-mb-xl">
        <div class="row items-center q-mb-md">
          <q-icon
            :name="currentModule.icon || 'school'"
            size="4rem"
            color="primary"
            class="q-mr-md"
          />
          <div>
            <div class="text-overline text-accent text-weight-bold">MÓDULO</div>
            <h1 class="text-h3 text-white q-my-none text-weight-bold">{{ currentModule.title }}</h1>
          </div>
        </div>

        <p class="text-h6 text-grey-4 module-description" v-if="currentModule.tooltip">
          {{ currentModule.tooltip }}
        </p>

        <!-- Stats / Metadata -->
        <div class="row q-gutter-md q-mt-md">
          <q-chip v-if="currentModule.difficulty" outline color="secondary" icon="network_check">
            {{ capitalize(currentModule.difficulty!) }}
          </q-chip>
          <q-chip v-if="currentModule.estimatedTime" outline color="accent" icon="schedule">
            {{ currentModule.estimatedTime }}
          </q-chip>
          <q-chip
            v-for="tag in currentModule.tags"
            :key="tag"
            color="dark"
            text-color="grey-4"
            size="sm"
          >
            #{{ tag }}
          </q-chip>
        </div>
      </div>

      <q-separator color="grey-8" class="q-mb-xl" />

      <!-- Lista de Lecciones -->
      <div class="lessons-grid">
        <div
          v-for="(lesson, index) in currentModule.children"
          :key="lesson.path"
          class="lesson-card-wrapper"
        >
          <q-card class="lesson-card cursor-pointer" @click="goToLesson(lesson.path)">
            <div class="lesson-status-line"></div>
            <q-card-section>
              <div class="text-overline text-primary q-mb-xs">LECCIÓN {{ index + 1 }}</div>
              <div class="text-h6 text-white q-mb-sm lesson-title">{{ lesson.title }}</div>
              <div class="text-caption text-grey-5 lesson-desc">
                {{ lesson.description || 'Explora este tema fundamental para dominar ROS 2.' }}
              </div>
            </q-card-section>

            <q-card-actions align="right">
              <q-btn flat color="primary" label="Comenzar" icon-right="arrow_forward" />
            </q-card-actions>
          </q-card>
        </div>
      </div>
    </div>

    <!-- State: Not Found -->
    <div v-else class="column flex-center full-height q-pa-xl text-center">
      <q-icon name="warning" size="4rem" color="warning" class="q-mb-md" />
      <h2 class="text-h4 text-white">Módulo no encontrado</h2>
      <p class="text-grey-4">No pudimos encontrar la información para esta ruta.</p>
      <q-btn to="/home" color="primary" label="Ir al Inicio" rounded unelevated class="q-mt-lg" />
    </div>
  </q-page>
</template>

<script setup lang="ts">
import { computed } from 'vue';
import { useRoute, useRouter } from 'vue-router';
import { courseStructure } from 'src/data/courseStructure';

const route = useRoute();
const router = useRouter();

// Encontrar el módulo actual basado en la URL
const currentModule = computed(() => {
  // La ruta suele ser /modulo-X. Buscamos en courseStructure.
  // route.path puede ser "/modulo-0"
  // courseStructure[i].path es "modulo-0"

  // Limpiamos el slash inicial para comparar
  const pathKey = route.path.replace(/^\//, '');

  return courseStructure.find((m) => m.path === pathKey);
});

const goToLesson = (lessonPath: string | undefined) => {
  if (!currentModule.value || !currentModule.value.path || !lessonPath) return;
  // Construimos la ruta completa: /modulo-0/01navsistemaPage
  // Ojo: definimos en routes.ts que las rutas hijas son relativas al padre?
  // En routes.ts actual: path: `${node.path}/${child.path}` -> "modulo-0/01navsistemaPage"
  // Así que navegamos a:
  const target = `/${currentModule.value.path}/${lessonPath}`;
  void router.push(target);
};

const capitalize = (s: string) => s.charAt(0).toUpperCase() + s.slice(1);
</script>

<style scoped>
.module-landing-page {
  background: #0f172a; /* Slate 900 base */
  min-height: 100vh;
}

.content-container {
  max-width: 1100px;
  margin: 0 auto;
}

.module-description {
  max-width: 800px;
  line-height: 1.6;
}

.lessons-grid {
  display: grid;
  grid-template-columns: repeat(auto-fill, minmax(300px, 1fr));
  gap: 24px;
}

.lesson-card {
  background: rgba(30, 41, 59, 0.5); /* Slate 800 transparent */
  border: 1px solid rgba(148, 163, 184, 0.1);
  border-radius: 16px;
  height: 100%;
  display: flex;
  flex-direction: column;
  justify-content: space-between;
  transition: all 0.3s ease;
  overflow: hidden;
  position: relative;
}

.lesson-card:hover {
  transform: translateY(-5px);
  background: rgba(30, 41, 59, 0.8);
  border-color: rgba(56, 189, 248, 0.5); /* Sky 400 */
  box-shadow: 0 10px 30px -10px rgba(0, 0, 0, 0.5);
}

/* Linea decorativa */
.lesson-status-line {
  height: 4px;
  background: linear-gradient(90deg, #38bdf8, #818cf8);
  width: 0%;
  transition: width 0.3s ease;
}

.lesson-card:hover .lesson-status-line {
  width: 100%;
}

.lesson-title {
  line-height: 1.3;
}

.lesson-desc {
  display: -webkit-box;
  -webkit-line-clamp: 3;
  line-clamp: 3;
  -webkit-box-orient: vertical;
  overflow: hidden;
}
</style>
