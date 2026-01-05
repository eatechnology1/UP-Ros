<template>
  <div class="video-block-wrapper">
    <!-- Contenedor del Video -->
    <div class="video-container">
      <iframe
        :src="src"
        :title="title || 'Video player'"
        frameborder="0"
        allow="
          accelerometer;
          autoplay;
          clipboard-write;
          encrypted-media;
          gyroscope;
          picture-in-picture;
        "
        allowfullscreen
        loading="lazy"
      />
    </div>

    <!-- Caption / Título (Opcional) -->
    <div v-if="title" class="video-caption">
      <q-icon name="play_circle_outline" size="xs" class="q-mr-xs" />
      {{ title }}
    </div>
  </div>
</template>

<script setup lang="ts">
defineProps<{
  src: string;
  title?: string; // Opcional: Descripción debajo del video
}>();
</script>

<style scoped>
.video-block-wrapper {
  width: 100%;
  margin-bottom: 2rem;
  display: flex;
  flex-direction: column;
  align-items: center;
}

.video-container {
  width: 100%;
  position: relative;
  /* Aspect Ratio 16:9 Mágico: Mantiene la proporción perfecta siempre */
  aspect-ratio: 16 / 9;

  background: rgba(15, 23, 42, 0.8); /* Fondo oscuro mientras carga */
  border-radius: 16px;
  overflow: hidden;
  box-shadow: 0 20px 40px -10px rgba(0, 0, 0, 0.5);
  border: 1px solid rgba(148, 163, 184, 0.1);
}

iframe {
  width: 100%;
  height: 100%;
  display: block;
}

/* Caption (Consistente con ImageBlock) */
.video-caption {
  margin-top: 12px;
  color: #94a3b8; /* Slate-400 */
  font-size: 0.9rem;
  display: flex;
  align-items: center;
  font-weight: 500;
}

/* RESPONSIVE */
/* En pantallas muy anchas, limitamos la altura para que no ocupe toda la vertical */
@media (min-width: 1200px) {
  .video-container {
    /* Opcional: Si no quieres videos gigantes en monitores 4K */
    max-height: 600px;
  }
}
</style>
