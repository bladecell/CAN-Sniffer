import '../app.css';
import { mount } from 'svelte';
import BackgroundPrototype from './BackgroundPrototype.svelte';

mount(BackgroundPrototype, { target: document.getElementById('app')! });
