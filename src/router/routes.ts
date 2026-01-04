import type { RouteRecordRaw } from 'vue-router';

const routes: RouteRecordRaw[] = [
  {
    path: '/',
    component: () => import('layouts/MainLayout.vue'),
    children: [
      { path: '', component: () => import('pages/IndexPage.vue') },
      { path: 'introduccion', component: () => import('pages/IntroduccionPage.vue') },
      { path: 'instalacion', component: () => import('pages/InstalacionPage.vue') },
      { path: 'fundamentos', component: () => import('pages/FundamentosPage.vue') },
      { path: 'ejemplos', component: () => import('pages/EjemplosPage.vue') },
      { path: 'simulacion', component: () => import('pages/SimulacionPage.vue') },
      { path: 'proyecto', component: () => import('pages/ProyectoPage.vue') },
      { path: 'glosario', component: () => import('pages/GlosarioPage.vue') },
      { path: 'creditos', component: () => import('pages/CreditosPage.vue') },
    ],
  },

  // Always leave this as last one
  {
    path: '/:catchAll(.*)*',
    component: () => import('pages/ErrorNotFound.vue'),
  },
];

export default routes;
