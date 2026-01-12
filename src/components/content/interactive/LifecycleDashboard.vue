<template>
  <div class="lifecycle-dashboard">
    <div class="dash-header">
      <div class="header-title">
        <q-icon name="settings_system_daydream" color="purple-4" />
        Lifecycle Node Manager
      </div>
      <div class="state-display" :class="currentState.toLowerCase()">State: {{ currentState }}</div>
    </div>

    <!-- STATE VISUALIZATION -->
    <div class="state-diagram">
      <div class="state-node unconfigured" :class="{ active: currentState === 'Unconfigured' }">
        Unconfigured
      </div>

      <div class="transition-arrow" :class="{ active: currentState === 'Inactive' }">↓</div>

      <div class="state-node inactive" :class="{ active: currentState === 'Inactive' }">
        Inactive
      </div>

      <div class="transition-arrow" :class="{ active: currentState === 'Active' }">↓</div>

      <div class="state-node active-state" :class="{ active: currentState === 'Active' }">
        Active
      </div>

      <div class="transition-arrow" :class="{ active: currentState === 'Finalized' }">↓</div>

      <div class="state-node finalized" :class="{ active: currentState === 'Finalized' }">
        Finalized
      </div>
    </div>

    <!-- CONTROLS -->
    <div class="controls-area">
      <div class="control-label">Available Transitions:</div>
      <div class="buttons-grid">
        <q-btn
          v-for="trans in availableTransitions"
          :key="trans.label"
          :label="trans.label"
          :color="trans.color"
          size="sm"
          unelevated
          class="trans-btn"
          @click="triggerTransition(trans)"
        />
        <q-btn
          v-if="availableTransitions.length === 0"
          label="Reset Sim"
          color="grey-8"
          size="sm"
          outline
          @click="reset"
        />
      </div>
    </div>

    <!-- LOGS -->
    <div class="console-log">
      <div v-for="(log, i) in logs" :key="i" class="log-line">
        <span class="log-time">[{{ log.time }}]</span>
        <span class="log-msg" :class="log.type">{{ log.msg }}</span>
      </div>
    </div>
  </div>
</template>

<script setup lang="ts">
import { ref, computed } from 'vue';

type State = 'Unconfigured' | 'Inactive' | 'Active' | 'Finalized';

interface Transition {
  label: string;
  target: State;
  color: string;
  callback: string;
}

const currentState = ref<State>('Unconfigured');
const logs = ref<{ time: string; msg: string; type: string }[]>([]);

const addLog = (msg: string, type = 'info') => {
  const time = new Date().toLocaleTimeString().split(' ')[0] || '';
  logs.value.unshift({ time, msg, type });
  if (logs.value.length > 5) logs.value.pop();
};

const transitions = computed<Transition[]>(() => {
  switch (currentState.value) {
    case 'Unconfigured':
      return [
        { label: 'configure', target: 'Inactive', color: 'blue', callback: 'on_configure()' },
        { label: 'shutdown', target: 'Finalized', color: 'red', callback: 'on_shutdown()' },
      ];
    case 'Inactive':
      return [
        { label: 'activate', target: 'Active', color: 'green', callback: 'on_activate()' },
        { label: 'cleanup', target: 'Unconfigured', color: 'orange', callback: 'on_cleanup()' },
        { label: 'shutdown', target: 'Finalized', color: 'red', callback: 'on_shutdown()' },
      ];
    case 'Active':
      return [
        {
          label: 'deactivate',
          target: 'Inactive',
          color: 'blue-grey',
          callback: 'on_deactivate()',
        },
        { label: 'shutdown', target: 'Finalized', color: 'red', callback: 'on_shutdown()' },
      ];
    case 'Finalized':
      return [];
    default:
      return [];
  }
});

const availableTransitions = computed(() => transitions.value);

const triggerTransition = (trans: Transition) => {
  addLog(`Triggering ${trans.label}...`, 'highlight');
  addLog(`Executing ${trans.callback}`);

  setTimeout(() => {
    currentState.value = trans.target;
    addLog(`Transitioned to ${trans.target}`, 'success');
  }, 500);
};

const reset = () => {
  currentState.value = 'Unconfigured';
  logs.value = [];
  addLog('System Reset', 'info');
};

// Init
addLog('Node instantiated. State: Unconfigured');
</script>

<style scoped>
.lifecycle-dashboard {
  background: rgba(15, 23, 42, 0.9);
  border: 1px solid rgba(148, 163, 184, 0.2);
  border-radius: 12px;
  overflow: hidden;
  max-width: 500px;
  margin: 2rem auto;
  font-family: 'Inter', sans-serif;
}

.dash-header {
  padding: 1rem;
  background: rgba(0, 0, 0, 0.4);
  display: flex;
  justify-content: space-between;
  align-items: center;
  border-bottom: 1px solid rgba(255, 255, 255, 0.1);
}

.header-title {
  color: #f1f5f9;
  font-weight: 700;
  display: flex;
  align-items: center;
  gap: 0.5rem;
}

.state-display {
  font-family: 'Fira Code', monospace;
  font-weight: 700;
  padding: 0.25rem 0.75rem;
  border-radius: 4px;
  font-size: 0.85rem;
}

.state-display.unconfigured {
  color: #94a3b8;
  background: rgba(148, 163, 184, 0.2);
}
.state-display.inactive {
  color: #60a5fa;
  background: rgba(96, 165, 250, 0.2);
}
.state-display.active {
  color: #4ade80;
  background: rgba(74, 222, 128, 0.2);
}
.state-display.finalized {
  color: #f87171;
  background: rgba(248, 113, 113, 0.2);
}

.state-diagram {
  padding: 2rem;
  display: flex;
  flex-direction: column;
  align-items: center;
  gap: 0.5rem;
  background: rgba(0, 0, 0, 0.2);
}

.state-node {
  width: 140px;
  padding: 0.75rem;
  text-align: center;
  border: 2px solid #334155;
  border-radius: 8px;
  color: #64748b;
  font-weight: 600;
  font-size: 0.9rem;
  transition: all 0.3s ease;
}

.state-node.active {
  border-color: currentColor;
  background: rgba(255, 255, 255, 0.05);
  box-shadow: 0 0 15px currentColor;
  transform: scale(1.05);
}

.state-node.unconfigured.active {
  color: #94a3b8;
  border-color: #94a3b8;
}
.state-node.inactive.active {
  color: #60a5fa;
  border-color: #60a5fa;
}
.state-node.active-state.active {
  color: #4ade80;
  border-color: #4ade80;
}
.state-node.finalized.active {
  color: #f87171;
  border-color: #f87171;
}

.transition-arrow {
  color: #334155;
  font-weight: bold;
}
.transition-arrow.active {
  color: #fff;
}

.controls-area {
  padding: 1.5rem;
  border-top: 1px solid rgba(255, 255, 255, 0.1);
}

.control-label {
  font-size: 0.75rem;
  color: #94a3b8;
  margin-bottom: 0.75rem;
  text-transform: uppercase;
}

.buttons-grid {
  display: flex;
  flex-wrap: wrap;
  gap: 0.75rem;
}

.trans-btn {
  font-family: 'Fira Code', monospace;
}

.console-log {
  background: #0f172a;
  padding: 1rem;
  font-family: 'Fira Code', monospace;
  font-size: 0.75rem;
  height: 120px;
  overflow-y: auto;
  display: flex;
  flex-direction: column;
  gap: 0.25rem;
}

.log-line {
  display: flex;
  gap: 0.75rem;
}

.log-time {
  color: #475569;
}
.log-msg {
  color: #cbd5e1;
}
.log-msg.highlight {
  color: #fde047;
}
.log-msg.success {
  color: #4ade80;
}
</style>
