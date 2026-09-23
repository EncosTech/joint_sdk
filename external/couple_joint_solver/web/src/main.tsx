import React, { useEffect, useState } from 'react';
import { createRoot } from 'react-dom/client';
import type { StoredConfig, WasmModule, WasmSolver } from './types';
import { defaultConfig, legacyStorageKeys, loadStoredConfig, storageKey } from './utils/storage';
import { createSolver, loadWasm, toErrorMessage } from './utils/wasm';
import { ConfigPage } from './pages/ConfigPage';
import { ViewerPage } from './pages/ViewerPage';
import './styles.css';

function App() {
  const [module, setModule] = useState<WasmModule | null>(null);
  const [solver, setSolver] = useState<WasmSolver | null>(null);
  const [config, setConfig] = useState<StoredConfig>(() => loadStoredConfig() ?? defaultConfig);
  const [mode, setMode] = useState<'loading' | 'config' | 'viewer'>('loading');
  const [error, setError] = useState('');

  useEffect(() => {
    loadWasm()
      .then((loadedModule) => {
        setModule(loadedModule);
        const stored = loadStoredConfig();
        if (!stored) {
          setMode('config');
          return;
        }
        const result = createSolver(loadedModule, stored.calibration);
        if (!result.solver) {
          setError(result.error);
          setMode('config');
          return;
        }
        setConfig(stored);
        setSolver(result.solver);
        setMode('viewer');
      })
      .catch((loadError: unknown) => {
        setError(toErrorMessage(loadError));
        setMode('config');
      });
  }, []);

  const saveConfig = (nextConfig: StoredConfig) => {
    if (!module) {
      setError('WASM module is not loaded.');
      return;
    }
    const result = createSolver(module, nextConfig.calibration);
    if (!result.solver) {
      setError(result.error);
      setMode('config');
      return;
    }
    setConfig(nextConfig);
    setSolver(result.solver);
    setError('');
    setMode('viewer');
  };

  const reset = () => {
    window.localStorage.removeItem(storageKey);
    legacyStorageKeys.forEach((key) => window.localStorage.removeItem(key));
    setSolver(null);
    setConfig(defaultConfig);
    setMode('config');
  };

  if (mode === 'loading') {
    return <main className="loading">正在加载 WASM 解算器</main>;
  }
  if (mode === 'config' || !solver) {
    return <ConfigPage initialConfig={config} error={error} onSave={saveConfig} />;
  }
  return <ViewerPage solver={solver} config={config} onReset={reset} />;
}

createRoot(document.getElementById('root') as HTMLElement).render(<App />);
