import React, { FormEvent, useRef, useState } from 'react';
import { Clipboard, ClipboardPaste, Download, Save, Upload } from 'lucide-react';
import type { AngleLimit, CalibrationInput, Point3, StoredConfig } from '../types';
import { rotationOrder } from '../utils/constants';
import { parseAndMigrateConfig, serializeConfig, storageKey } from '../utils/storage';
import { PointFields } from '../components/PointFields';

export function ConfigPage({
  initialConfig,
  error,
  onSave,
}: {
  initialConfig: StoredConfig;
  error: string;
  onSave: (config: StoredConfig) => void;
}) {
  const [config, setConfig] = useState<StoredConfig>(initialConfig);
  const [localError, setLocalError] = useState('');
  const fileInputRef = useRef<HTMLInputElement | null>(null);
  const calibration = config.calibration;

  const updatePoint = (key: keyof CalibrationInput, point: Point3) => {
    setConfig({ ...config, calibration: { ...calibration, [key]: point } });
  };

  const updateLimit = (key: 'rollLimit' | 'pitchLimit', axis: keyof AngleLimit, value: string) => {
    setConfig({ ...config, [key]: { ...config[key], [axis]: value === '' ? null : Number(value) } });
  };

  const updateRotationOrder = (value: string) => {
    const order = Number(value);
    setConfig({ ...config, calibration: { ...calibration, rotationOrder: order } });
  };

  const applyImportedConfig = (nextConfig: StoredConfig) => {
    setConfig(nextConfig);
    setLocalError('');
    window.localStorage.setItem(storageKey, serializeConfig(nextConfig));
    onSave(nextConfig);
  };

  const importRawConfig = (raw: string) => {
    const nextConfig = parseAndMigrateConfig(raw);
    if (!nextConfig) {
      setLocalError('配置导入失败：JSON 格式或字段不完整。');
      return;
    }
    applyImportedConfig(nextConfig);
  };

  const exportConfigFile = () => {
    const blob = new Blob([serializeConfig(config)], { type: 'application/json' });
    const url = URL.createObjectURL(blob);
    const link = document.createElement('a');
    link.href = url;
    link.download = 'couple-joint-config.json';
    link.click();
    window.setTimeout(() => URL.revokeObjectURL(url), 0);
  };

  const copyConfig = async () => {
    try {
      await navigator.clipboard.writeText(serializeConfig(config));
      setLocalError('');
    } catch {
      setLocalError('复制失败：当前浏览器不允许访问剪贴板。');
    }
  };

  const importClipboard = async () => {
    try {
      importRawConfig(await navigator.clipboard.readText());
    } catch {
      setLocalError('剪贴板导入失败：当前浏览器不允许读取剪贴板。');
    }
  };

  const importConfigFile = async (event: React.ChangeEvent<HTMLInputElement>) => {
    const file = event.target.files?.[0];
    event.target.value = '';
    if (!file) {
      return;
    }
    importRawConfig(await file.text());
  };

  const submit = (event: FormEvent) => {
    event.preventDefault();
    window.localStorage.setItem(storageKey, serializeConfig(config));
    onSave(config);
  };

  return (
    <main className="config-page">
      <form className="config-panel" onSubmit={submit}>
        <div className="panel-heading">
          <h1>六连杆闭链机构参数</h1>
          <div className="heading-actions">
            <button type="button" className="secondary-action-button" onClick={exportConfigFile}>
              <Download size={18} />
              导出文件
            </button>
            <button type="button" className="secondary-action-button" onClick={() => fileInputRef.current?.click()}>
              <Upload size={18} />
              导入文件
            </button>
            <button type="button" className="secondary-action-button" onClick={copyConfig}>
              <Clipboard size={18} />
              复制 JSON
            </button>
            <button type="button" className="secondary-action-button" onClick={importClipboard}>
              <ClipboardPaste size={18} />
              粘贴导入
            </button>
            <button type="submit" className="primary-button">
              <Save size={18} />
              保存并进入
            </button>
          </div>
        </div>
        <input ref={fileInputRef} className="hidden-file-input" type="file" accept="application/json,.json" onChange={importConfigFile} />
        {localError || error ? <div className="error-banner">{localError || error}</div> : null}
        <section>
          <h2>初始关键点</h2>
          <PointFields label="左转动副 O1" value={calibration.leftJointCenter} onChange={(p) => updatePoint('leftJointCenter', p)} />
          <PointFields label="右转动副 O2" value={calibration.rightJointCenter} onChange={(p) => updatePoint('rightJointCenter', p)} />
          <PointFields label="左平面端点 P1" value={calibration.leftPlanarEndpoint} onChange={(p) => updatePoint('leftPlanarEndpoint', p)} />
          <PointFields label="右平面端点 P2" value={calibration.rightPlanarEndpoint} onChange={(p) => updatePoint('rightPlanarEndpoint', p)} />
          <PointFields label="Roll 旋转中心" value={calibration.rollCenter} onChange={(p) => updatePoint('rollCenter', p)} />
          <PointFields label="Pitch 旋转中心" value={calibration.pitchCenter} onChange={(p) => updatePoint('pitchCenter', p)} />
          <PointFields label="左空间端点 S1" value={calibration.leftSpatialEndpoint} onChange={(p) => updatePoint('leftSpatialEndpoint', p)} />
          <PointFields label="右空间端点 S2" value={calibration.rightSpatialEndpoint} onChange={(p) => updatePoint('rightSpatialEndpoint', p)} />
        </section>
        <section className="rotation-order-section">
          <h2>姿态旋转顺序</h2>
          <div className="rotation-order-control" role="radiogroup" aria-label="Pitch / Roll 顺序">
            <button
              type="button"
              className={
                calibration.rotationOrder === rotationOrder.pitchThenRoll
                  ? 'rotation-order-option selected'
                  : 'rotation-order-option'
              }
              role="radio"
              aria-checked={calibration.rotationOrder === rotationOrder.pitchThenRoll}
              onClick={() => updateRotationOrder(String(rotationOrder.pitchThenRoll))}
            >
              <span className="rotation-order-main">先 Pitch 后 Roll</span>
              <span className="rotation-order-sub">Pitch -&gt; Roll</span>
            </button>
            <button
              type="button"
              className={
                calibration.rotationOrder === rotationOrder.rollThenPitch
                  ? 'rotation-order-option selected'
                  : 'rotation-order-option'
              }
              role="radio"
              aria-checked={calibration.rotationOrder === rotationOrder.rollThenPitch}
              onClick={() => updateRotationOrder(String(rotationOrder.rollThenPitch))}
            >
              <span className="rotation-order-main">先 Roll 后 Pitch</span>
              <span className="rotation-order-sub">Roll -&gt; Pitch</span>
            </button>
          </div>
        </section>
        <section className="limit-grid">
          <h2>Roll / Pitch 限制</h2>
          <label>
            <span>Roll min deg</span>
            <input
              type="number"
              value={config.rollLimit.min ?? ''}
              onChange={(event) => updateLimit('rollLimit', 'min', event.target.value)}
            />
          </label>
          <label>
            <span>Roll max deg</span>
            <input
              type="number"
              value={config.rollLimit.max ?? ''}
              onChange={(event) => updateLimit('rollLimit', 'max', event.target.value)}
            />
          </label>
          <label>
            <span>Pitch min deg</span>
            <input
              type="number"
              value={config.pitchLimit.min ?? ''}
              onChange={(event) => updateLimit('pitchLimit', 'min', event.target.value)}
            />
          </label>
          <label>
            <span>Pitch max deg</span>
            <input
              type="number"
              value={config.pitchLimit.max ?? ''}
              onChange={(event) => updateLimit('pitchLimit', 'max', event.target.value)}
            />
          </label>
        </section>
      </form>
    </main>
  );
}
