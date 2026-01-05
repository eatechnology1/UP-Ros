<template>
  <div class="image-block-wrapper">
    <!-- Imagen principal -->
    <div
      class="image-container"
      :class="{ 'image-zoomable': zoomable }"
      @click="toggleZoom"
      role="button"
      :aria-label="zoomable ? 'Ampliar imagen' : alt"
      :tabindex="zoomable ? 0 : -1"
      @keydown.enter="toggleZoom"
    >
      <img
        ref="imageRef"
        :src="src"
        :alt="alt"
        :loading="lazy ? 'lazy' : undefined"
        draggable="false"
      />

      <!-- Overlay zoom (solo visual) -->
      <div v-if="zoomable" class="zoom-hint">
        <q-icon name="zoom_in" />
        <span class="zoom-text">Ampliar</span>
      </div>
    </div>

    <!-- Caption / Créditos -->
    <div v-if="caption || credit" class="image-meta">
      <div v-if="caption" class="caption-text">{{ caption }}</div>
      <div v-if="credit" class="credit-text">
        <q-icon name="photo_camera" size="xs" class="q-mr-xs" />
        {{ credit }}
      </div>
    </div>

    <!-- Modal zoom (Fullscreen) -->
    <q-dialog
      v-model="zoomedDialog"
      transition-show="scale"
      transition-hide="scale"
      backdrop-filter="blur(4px)"
    >
      <div class="zoom-modal-container">
        <!-- Botón cerrar flotante -->
        <q-btn
          icon="close"
          flat
          round
          dense
          color="white"
          class="zoom-close-btn"
          @click="closeZoom"
          aria-label="Cerrar zoom"
        />

        <img :src="src" :alt="alt" class="zoom-image" />

        <!-- Caption en zoom también -->
        <div v-if="caption" class="zoom-caption">
          {{ caption }}
        </div>
      </div>
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

function toggleZoom() {
  if (props.zoomable) {
    zoomedDialog.value = true;
  }
}

function closeZoom() {
  zoomedDialog.value = false;
}

onMounted(() => {
  if (imageRef.value) {
    imageRef.value.addEventListener('load', () => emit('load'));
  }
});
</script>

<style scoped>
.image-block-wrapper {
  width: 100%;
  margin-bottom: 2rem;
  display: flex;
  flex-direction: column;
  align-items: center;
}

/* CONTENEDOR IMAGEN */
.image-container {
  position: relative;
  border-radius: 16px;
  overflow: hidden;
  box-shadow: 0 10px 30px -10px rgba(0, 0, 0, 0.5);
  border: 1px solid rgba(148, 163, 184, 0.1);
  background: rgba(15, 23, 42, 0.5); /* Placeholder oscuro */
  display: inline-block; /* Ajustarse al tamaño de la imagen */
  max-width: 100%;
}

.image-zoomable {
  cursor: zoom-in;
}

.image-container img {
  display: block;
  max-width: 100%;
  height: auto;
  transition: transform 0.3s ease;
}

/* HINT ZOOM */
.zoom-hint {
  position: absolute;
  inset: 0;
  background: rgba(15, 23, 42, 0.4);
  display: flex;
  align-items: center;
  justify-content: center;
  gap: 8px;
  opacity: 0;
  transition: opacity 0.2s ease;
  color: white;
  font-weight: 500;
  backdrop-filter: blur(2px);
}

.image-zoomable:hover .zoom-hint {
  opacity: 1;
}

/* CAPTION & CREDIT */
.image-meta {
  margin-top: 12px;
  text-align: center;
  max-width: 90%;
}

.caption-text {
  color: #e2e8f0;
  font-size: 0.95rem;
  line-height: 1.5;
  margin-bottom: 4px;
}

.credit-text {
  color: #94a3b8;
  font-size: 0.75rem;
  font-style: italic;
  display: flex;
  align-items: center;
  justify-content: center;
}

/* MODAL ZOOM (ESTILO LIBRE SIN CARD DE QUASAR) */
.zoom-modal-container {
  position: relative;
  max-width: 95vw;
  max-height: 95vh;
  display: flex;
  flex-direction: column;
  align-items: center;
  justify-content: center;
  outline: none;
}

.zoom-image {
  max-width: 100%;
  max-height: 85vh; /* Deja espacio para caption/botón */
  object-fit: contain;
  border-radius: 8px;
  box-shadow: 0 20px 50px rgba(0, 0, 0, 0.8);
}

.zoom-close-btn {
  position: absolute;
  top: -40px;
  right: 0;
  background: rgba(255, 255, 255, 0.1);
}

.zoom-caption {
  margin-top: 16px;
  color: white;
  background: rgba(0, 0, 0, 0.6);
  padding: 8px 16px;
  border-radius: 20px;
  font-size: 0.9rem;
}

/* RESPONSIVE */
@media (max-width: 600px) {
  .zoom-close-btn {
    top: 10px;
    right: 10px;
    background: rgba(0, 0, 0, 0.5);
    z-index: 10;
  }
}
</style>
