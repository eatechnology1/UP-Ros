<template>
  <q-page padding>
    <content-viewer :section-id="id" />
  </q-page>
</template>

<script setup lang="ts">
import { useRoute } from 'vue-router';
import ContentViewer from 'components/ContentViewer.vue';

const route = useRoute();
const id = route.params.id as string;
</script>
