<template>
  <div class="text-block" :class="{ 'text-block-highlighted': highlighted }">
    <slot />
  </div>
</template>

<script setup lang="ts">
defineProps<{
  highlighted?: boolean;
}>();
</script>

<style scoped>
.text-block {
  max-width: 900px;
  margin: 32px auto;
  padding: 28px;
  background: rgba(15, 23, 42, 0.85);
  backdrop-filter: blur(16px);
  -webkit-backdrop-filter: blur(16px);
  border: 1px solid rgba(148, 163, 184, 0.25);
  border-radius: 20px;
  box-shadow:
    0 20px 60px rgba(15, 23, 42, 0.75),
    inset 0 1px 0 rgba(255, 255, 255, 0.06);
  transition: all 0.3s cubic-bezier(0.4, 0, 0.2, 1);
}

.text-block:hover {
  transform: translateY(-2px);
  box-shadow:
    0 28px 80px rgba(15, 23, 42, 0.85),
    inset 0 1px 0 rgba(255, 255, 255, 0.08);
}

.text-block-highlighted {
  border-left: 4px solid #6ecbff;
  background: rgba(110, 203, 255, 0.08);
  border-color: rgba(110, 203, 255, 0.3);
}

.text-block :deep(strong) {
  color: #ffffff !important;
  font-weight: 600 !important;
}

.text-block :deep(em) {
  color: #cbd5f5 !important;
  font-style: italic !important;
}

.text-block :deep(code) {
  background: rgba(110, 203, 255, 0.15);
  color: #6ecbff;
  padding: 2px 6px;
  border-radius: 4px;
  font-size: 0.9em;
  font-family: 'Fira Code', monospace;
}

@media (max-width: 768px) {
  .text-block {
    margin: 24px 16px;
    padding: 20px;
  }
}
</style>
