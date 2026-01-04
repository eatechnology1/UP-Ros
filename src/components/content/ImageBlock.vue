<template>
  <div class="image-block-wrapper">
    <!-- Imagen principal -->
    <div class="image-container" :class="{ 'image-zoomable': zoomable }" @click="toggleZoom">
      <img
        ref="imageRef"
        :src="src"
        :alt="alt"
        :loading="lazy ? 'lazy' : undefined"
        draggable="false"
      />

      <!-- Overlay zoom (solo desktop) -->
      <div v-if="zoomable && !zoomed" class="zoom-hint">
        <q-icon name="zoom_in" />
        <span class="zoom-text">Click para ampliar</span>
      </div>
    </div>

    <!-- Caption -->
    <div v-if="caption || credit" class="image-caption">
      <div class="caption-text">{{ caption }}</div>
      <div v-if="credit" class="image-credit">📷 {{ credit }}</div>
    </div>

    <!-- Modal zoom (fullscreen) -->
    <q-dialog v-model="zoomedDialog" persistent>
      <q-card class="zoom-modal">
        <q-card-section class="row justify-end q-pa-none">
          <q-btn icon="close" flat round dense @click="closeZoom" aria-label="Cerrar imagen" />
        </q-card-section>
        <q-card-section class="zoom-content q-pa-none">
          <img :src="src" :alt="alt" class="zoom-image" />
        </q-card-section>
      </q-card>
    </q-dialog>
  </div>
</template>

<script setup lang="ts">
import { ref, onMounted } from 'vue';

const props = withDefaults(
  defineProps<{
    src: string;
    alt?: string;
    caption?: string;
    credit?: string;
    zoomable?: boolean;
    lazy?: boolean;
  }>(),
  {
    alt: 'Imagen ilustrativa',
    zoomable: true,
    lazy: true,
  },
);

const emit = defineEmits<{
  (e: 'load'): void;
}>();

const imageRef = ref<HTMLImageElement>();
const zoomedDialog = ref(false);
const zoomed = ref(false);

function toggleZoom() {
  if (props.zoomable) {
    zoomedDialog.value = true;
    zoomed.value = true;
  }
}

function closeZoom() {
  zoomedDialog.value = false;
  zoomed.value = false;
}

onMounted(() => {
  if (imageRef.value) {
    imageRef.value.addEventListener('load', () => emit('load'));
  }
});
</script>

<style scoped>
/* ... MISMO CSS ANTERIOR SIN CAMBIOS ... */
.image-block-wrapper {
  max-width: 900px;
  margin: 40px auto;
}

.image-container {
  position: relative;
  border-radius: 20px;
  overflow: hidden;
  cursor: pointer;
  transition: all 0.3s cubic-bezier(0.4, 0, 0.2, 1);
  backdrop-filter: blur(2px);
  box-shadow:
    0 20px 60px rgba(15, 23, 42, 0.7),
    0 0 0 1px rgba(255, 255, 255, 0.05);
}

.image-container:hover {
  transform: translateY(-4px);
  box-shadow:
    0 32px 80px rgba(15, 23, 42, 0.85),
    0 0 0 1px rgba(110, 203, 255, 0.2);
}

.image-container img {
  display: block;
  width: 100%;
  height: auto;
  transition: transform 0.3s ease;
}

/* Resto del CSS igual... */
/* FIX ZOOM MODAL - Imagen completa sin recorte */
.zoom-image {
  width: 100%;
  height: 100%;
  object-fit: contain !important; /* ← ESTO SOLO */
  object-position: center;
  background: rgba(15, 23, 42, 0.95); /* Fondo oscuro suave */
}
.zoom-modal {
  max-width: 90vw;
  max-height: 90vh;
  border-radius: 16px;
  overflow: hidden;
  backdrop-filter: blur(16px);
  -webkit-backdrop-filter: blur(16px);
  background: rgba(15, 23, 42, 0.95);
  box-shadow:
    0 32px 80px rgba(15, 23, 42, 0.85),
    inset 0 1px 0 rgba(255, 255, 255, 0.06);
}
</style>
