// 点云作业台端到端冒烟（阶段 1）：真实路由 + 作业台壳，后端用模拟接口，任务包文件来自 tools/testdata 生成器。
// 用法：
//   1) 在 annotation-platform/labeling-admin-web 下启动开发服务器：npx vite --port 5199
//   2) python3 tools/testdata/gen_packages.py --out /tmp/pc-pkgs --packages 1 --frames 20
//   3) PWG=<playwright 所在 node_modules> node tools/e2e/workbench-smoke.mjs <截图目录> /tmp/pc-pkgs/PCTEST_0001 [皮肤ID]
// 覆盖：合并帧加载、圈选 / 框选 / 移除区域 / 锁定、撤销重做、自动保存、单帧立方体新建、旋转手柄、编辑模式（正视 / 左视 / 俯视）、
// 相机图片、快捷键窗口、提交校验（必填属性、场景属性）、提交后返回列表；并检查作业台壳没有任何保存 / 提交调用。
import fs from 'node:fs'
import path from 'node:path'
const { chromium } = await import(process.env.PWG + '/playwright/index.mjs')
const out = process.argv[2]
const PKG_DIR = process.argv[3]
const THEME = process.argv[4] || ''
const pkgJson = JSON.parse(fs.readFileSync(path.join(PKG_DIR, 'package.json'), 'utf8'))
const browser = await chromium.launch({ args: ['--use-gl=swiftshader', '--enable-unsafe-swiftshader', '--ignore-gpu-blocklist'] })
const ok = (data) => ({ status: 200, contentType: 'application/json', body: JSON.stringify({ code: 0, message: 'ok', data }) })
const names = JSON.parse(fs.readFileSync(path.join(PKG_DIR, '..', 'classes.json'), 'utf8'))
const classes = names.map((c) => ({ classId: c.englishName, name: c.name, color: c.color, categoryId: c.categoryId, geometries: ['cuboid'],
  attributes: c.englishName === 'car' ? [{ attrId: 'occ', name: '遮挡', type: 'radio', options: ['无', '部分', '严重'], required: true }] : [] }))
const classModes = classes.map((c) => ({ classId: c.classId, modes: c.classId === 'car' ? ['cuboid', 'point_seg'] : ['point_seg'] }))
const index = {
  manifestId: 5, packageId: pkgJson.packageId, single: false, sensors: pkgJson.sensors,
  frames: pkgJson.frames.map((f, i) => ({
    index: i, id: f.id, timestampUs: f.timestampUs, pose: f.pose,
    lidar: Object.entries(f.lidar).map(([sensor, key]) => ({ sensor, key, url: '/pkg/' + key })),
    cameras: Object.entries(f.cameras).map(([sensor, c]) => ({ sensor, key: c.key, url: '/pkg/' + c.key, timestampUs: c.timestampUs })),
  })),
}
const page = await browser.newPage({ viewport: { width: 1680, height: 960 } })
const calls = []
const errors = []
const versions = {}
page.on('pageerror', (e) => errors.push(e.message))
page.on('console', (m) => { if (m.type() === 'error') errors.push('console: ' + m.text()) })
await page.route((u) => u.pathname.startsWith('/pkg/'), (route) => {
  const rel = decodeURIComponent(new URL(route.request().url()).pathname.slice(5))
  return route.fulfill({ status: 200, path: path.join(PKG_DIR, rel) })
})
await page.route((u) => u.pathname.startsWith('/api/'), async (route) => {
  const req = route.request()
  const p = new URL(req.url()).pathname
  const body = req.postData()
  calls.push(req.method() + ' ' + p + (body ? ' ' + body.slice(0, 300) : ''))
  if (p === '/api/system/auth/profile') return route.fulfill(ok({ id: 7, principalKind: 'worker', permissionCodes: [], roleCodes: [] }))
  if (p === '/api/labeling/task/items/1/workbench') return route.fulfill(ok({
    item: { id: 1, taskId: 3, manifestId: 5, status: 'doing', assigneeId: 7 },
    task: { id: 3, taskCode: 'T3', projectId: 1, name: '4D分割测试', stage: 'annotate' },
    manifest: { id: 5, manifestCode: pkgJson.packageId, projectId: 1, datasetId: 2, mediaType: 'lidar', previewUrl: index.frames[0].lidar[0].url, annotationsJson: '' },
    classes, readonly: false, annotationType: 'cuboid',
    attributes: [
      { attrId: 'weather', name: '天气', type: 'radio', options: ['晴天', '雨天', '雾天'], required: true, scope: 'image' },
      { attrId: 'time', name: '时间', type: 'radio', options: ['白天', '夜晚'], scope: 'image' },
    ],
  }))
  if (p === '/api/labeling/pointcloud/projects/1/config') return route.fulfill(ok({ projectId: 1, classModes, settleBy: 'frame' }))
  if (p === '/api/labeling/pointcloud/items/1/package') return route.fulfill(ok(index))
  if (p === '/api/labeling/pointcloud/items/1/package/result') return route.fulfill(ok({ parts: [] }))
  if (p.startsWith('/api/labeling/pointcloud/items/1/package/')) {
    const parts = JSON.parse(body).parts.map((x) => ({ frameIndex: x.frameIndex, version: (versions[x.frameIndex] = (versions[x.frameIndex] || 0) + 1) }))
    return route.fulfill(ok(p.endsWith('/save') ? { parts, savedAt: new Date().toISOString() } : true))
  }
  return route.fulfill(ok(null))
})
const BASE = process.env.BASE_URL || 'http://localhost:5199'
await page.goto(BASE + '/login')
await page.evaluate((theme) => { localStorage.clear(); localStorage.setItem('oa_token', 'test'); localStorage.setItem('oa_platform', 'worker'); if (theme) localStorage.setItem('lbl-admin-theme', theme) }, THEME)
const t0 = Date.now()
await page.goto(BASE + '/workbench/1')
await page.waitForFunction(() => document.querySelector('.pcw-seg button.on')?.textContent === '合并帧' && !document.querySelector('.pcw-loading'), null, { timeout: 90000 })
await page.waitForTimeout(600)
const log = (...a) => console.log(...a)
log('loaded merged in', Date.now() - t0, 'ms | render:', (await page.textContent('.pcw-render')).replace(/\s+/g, ' '), '| tiles:', await page.locator('.pcw-frames .tile').count())
log('primary color:', await page.evaluate(() => getComputedStyle(document.querySelector('.pcw-seg button.on')).backgroundColor))
await page.screenshot({ path: out + '/n1-merged.png' })

const canvas = page.locator('canvas.pcv-canvas')
const box = await canvas.boundingBox()
const cx = box.x + box.width / 2, cy = box.y + box.height / 2
// 顶视角后圈选画面下半部（路面附近）
await page.keyboard.press('t'); await page.waitForTimeout(500)
await page.keyboard.press('q')
await page.mouse.move(cx - 300, cy + 40); await page.mouse.down()
for (const [dx, dy] of [[300, 40], [300, 200], [-300, 200], [-300, 40]]) await page.mouse.move(cx + dx, cy + dy, { steps: 5 })
await page.mouse.up(); await page.waitForTimeout(300)
log('picker open:', await page.locator('.pcw-picker').isVisible())
await page.keyboard.type('地面'); await page.keyboard.press('Enter'); await page.waitForTimeout(400)
const counts = async () => JSON.stringify(await page.$$eval('.class-row', (e) => e.slice(0, 3).map((x) => x.querySelector('.name').textContent + ':' + x.querySelector('.cnt').textContent)))
log('after lasso:', await counts(), '| unlabeled:', await page.textContent('.total .muted'))
// 锁定“地面”后用框选整片赋人行道：地面点不应改变
await page.locator('.class-row').first().locator('.ic').nth(1).click()
await page.keyboard.press('b')
await page.mouse.move(cx - 350, cy - 250); await page.mouse.down(); await page.mouse.move(cx + 350, cy + 300, { steps: 6 }); await page.mouse.up(); await page.waitForTimeout(300)
await page.keyboard.type('人行道'); await page.keyboard.press('Enter'); await page.waitForTimeout(400)
log('locked ground + rect sidewalk:', await counts())
await page.keyboard.press('Control+z'); await page.waitForTimeout(300)
log('undo:', await counts())
await page.keyboard.press('Control+y'); await page.waitForTimeout(300)
log('redo:', await counts())
// 移除区域
await page.keyboard.press('-')
await page.mouse.move(cx - 60, cy - 60); await page.mouse.down(); await page.mouse.move(cx + 60, cy + 60, { steps: 4 }); await page.mouse.up(); await page.waitForTimeout(300)
log('remove rect:', await counts())
await page.keyboard.press('=')
await page.screenshot({ path: out + '/n2-seg.png' })
// 自动保存
await page.waitForTimeout(3600)
const save = calls.filter((c) => c.includes('/package/save'))
log('autosave calls:', save.length, '| state:', await page.textContent('.save'))

// 单帧 + 立方体
await page.locator('.pcw-seg button', { hasText: '单帧' }).click(); await page.waitForTimeout(300)
await page.locator('.pcw-frames .tile').nth(4).click(); await page.waitForTimeout(1200)
await page.keyboard.press('Alt+1'); await page.waitForTimeout(200)
await page.keyboard.press('t'); await page.waitForTimeout(500)
await page.keyboard.press('n')
await page.mouse.move(cx + 40, cy - 30); await page.mouse.down(); await page.mouse.move(cx + 110, cy, { steps: 5 }); await page.mouse.up(); await page.waitForTimeout(400)
log('boxes in frame 5:', await page.locator('.obj-row').count(), '| tool:', await page.locator('.pcw-toolbar .grp:nth-child(2) button.on').textContent())
const shape0 = await page.evaluate(() => document.querySelector('.field.nums')?.textContent?.replace(/\s+/g, ' '))
log('selected:', shape0)
// 旋转手柄：开发环境下作业台暴露 window.__pcViewport.handleScreen() 给出手柄的屏幕位置
const yawBefore = shape0.match(/([0-9.]+)°/)[1]
const hs = await page.evaluate(() => window.__pcViewport.handleScreen())
const r0 = await canvas.boundingBox()
const hx = r0.x + hs[0], hy = r0.y + hs[1]
await page.mouse.move(hx, hy); await page.mouse.down()
await page.mouse.move(hx + 30, hy + 45, { steps: 6 }); await page.mouse.up(); await page.waitForTimeout(200)
const now = (await page.evaluate(() => document.querySelector('.field.nums')?.textContent || '')).match(/([0-9.]+)°/)
let rotated = now && now[1] !== yawBefore
log('rotation handle: yaw', yawBefore, '→', now && now[1])
if (!rotated) log('rotation handle not hit by probe')
// 编辑模式
await page.keyboard.press('Enter'); await page.waitForTimeout(500)
log('edit mode views:', JSON.stringify(await page.$$eval('.pcw-edit canvas', (e) => e.map((x) => x.className))))
const front = await page.locator('.pcw-edit canvas.tv-front').boundingBox()
await page.mouse.move(front.x + front.width / 2, front.y + front.height / 2); await page.mouse.down(); await page.mouse.move(front.x + front.width / 2 + 20, front.y + front.height / 2 - 15, { steps: 4 }); await page.mouse.up(); await page.waitForTimeout(300)
log('after front-view drag:', await page.evaluate(() => document.querySelector('.field.nums')?.textContent?.replace(/\s+/g, ' ')))
await page.screenshot({ path: out + '/n3-edit.png' })
await page.keyboard.press('Escape'); await page.waitForTimeout(200)
// 属性必填、相机、场景属性、快捷键窗口
await page.keyboard.press('c'); await page.waitForTimeout(800)
log('camera images:', await page.locator('.cam-grid img').count())
await page.screenshot({ path: out + '/n4-camera.png' })
await page.keyboard.press('c')
await page.keyboard.press('?'); await page.waitForTimeout(400)
log('hotkey dialog rows:', await page.locator('.hk-row').count())
await page.screenshot({ path: out + '/n5-hotkeys.png' })
await page.keyboard.press('Escape'); await page.waitForTimeout(300)
await page.waitForTimeout(400)
// 提交：期望必填属性与场景属性阻止
await page.keyboard.press('Control+Enter'); await page.waitForTimeout(500)
log('blocked:', (await page.textContent('.el-message-box__message'))?.replace(/\s+/g, ' ').slice(0, 160))
await page.locator('.el-message-box__btns button').last().click(); await page.waitForTimeout(300)
await page.locator('.field .el-select').nth(1).click(); await page.waitForTimeout(200)
await page.locator('.el-select-dropdown__item:visible', { hasText: '部分' }).click(); await page.waitForTimeout(200)
await page.locator('.pcw-rail button', { hasText: '景' }).click(); await page.waitForTimeout(300)
await page.locator('.scene-grid .el-select').first().click(); await page.waitForTimeout(200)
await page.locator('.el-select-dropdown__item:visible', { hasText: '晴天' }).click(); await page.waitForTimeout(200)
await page.locator('body').click({ position: { x: 5, y: 5 } }).catch(() => {})
await page.locator('.pcw-btn.primary', { hasText: '提交' }).click(); await page.waitForTimeout(600)
log('soft:', (await page.textContent('.el-message-box__message').catch(() => ''))?.replace(/\s+/g, ' ').slice(0, 200))
await page.locator('.el-message-box__btns button', { hasText: '仍要继续' }).click().catch(() => {})
await page.waitForTimeout(1200)
const sub = calls.find((c) => c.includes('/package/submit'))
log('submit sent:', !!sub, '| url after:', new URL(page.url()).pathname)
log('shell writes:', calls.filter((c) => /\/task\/items\/1\/(draft|submit|accept|reject|review-notes)/.test(c)).length, '| errors:', errors.join(' ; ') || 'none')
fs.writeFileSync(out + '/calls.txt', calls.join('\n'))
await browser.close()
