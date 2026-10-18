import { defineConfig } from 'vite'
import { svelte } from '@sveltejs/vite-plugin-svelte'
import { viteSingleFile } from "vite-plugin-singlefile"
import path from 'path';
import { execFileSync } from 'node:child_process';
import { readFileSync, writeFileSync, rmSync } from 'node:fs';
import { gzipSync } from 'node:zlib';

// 7-Zip's deflate beats zlib -9 by ~4%; falls back to zlib when 7z is missing.
const gzipIndex = () => ({
	name: 'gzip-index',
	apply: 'build',
	closeBundle() {
		const html = path.resolve(__dirname, 'dist/index.html');
		const gz = html + '.gz';
		try {
			rmSync(gz, { force: true });
			execFileSync('7z', ['a', '-tgzip', '-mx=9', '-mpass=15', '-mfb=258', gz, html], { stdio: 'ignore' });
		} catch {
			writeFileSync(gz, gzipSync(readFileSync(html), { level: 9 }));
		}
		rmSync(html);
	}
});

const DEVICE_HOST = process.env.CAN_SNIFFER_TARGET ?? 'can-sniffer.local';
const DEVICE_HTTP = `http://${DEVICE_HOST}`;
const DEVICE_WS = `ws://${DEVICE_HOST}`;
// The firmware's own name, even when CAN_SNIFFER_TARGET points elsewhere.
const DEVICE_ORIGIN = 'http://can-sniffer.local';

// Runs before the request is forwarded (http-proxy's proxyReq event is too late: headers are already sent).
// Requests without an Origin (curl, the bundled tools) pass the device check unchanged.
const rewriteOrigin = (req) => {
	if (req.headers.origin) req.headers.origin = DEVICE_ORIGIN;
};

export default defineConfig({
	plugins: [
		svelte(),
		viteSingleFile(),
		gzipIndex()
	],
	build: {
		reportCompressedSize: false,
		cssCodeSplit: false,
		assetsInlineLimit: 100000000, // This will inline the favicon automatically
	},
	resolve: {
		alias: {
			$lib: path.resolve(__dirname, './src/lib')
		}
	},
	server: {
		host: true,

		// The firmware accepts only its own Host and Origin (docs/api.md, Host, Origin and Content-Type guard).
		// changeOrigin sets Host to the target; the Origin of the dev page is replaced with the device's.
		// CAN_SNIFFER_TARGET overrides the device address (used to test the proxy against a mock).
		proxy: {
			'/api': {
				target: DEVICE_HTTP,
				changeOrigin: true,
				bypass: rewriteOrigin,
			},
			'/ws': {
				target: DEVICE_WS,
				ws: true,
				secure: false,
				changeOrigin: true,
				bypass: rewriteOrigin,
			}
		}
	}
})
