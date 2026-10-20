/// <reference types="vite/client" />

declare module '/wasm/couple_joint_solver.js' {
  const createModule: () => Promise<unknown>;
  export default createModule;
}
