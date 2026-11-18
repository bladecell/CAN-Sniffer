import { clsx, type ClassValue } from 'clsx';
import { twMerge } from 'tailwind-merge';
import type { Snippet } from 'svelte';

export type WithElementRef<T> = T & { ref?: HTMLElement | null };
export type WithoutChild<T> = Omit<T, 'child'> & { child?: Snippet };
export type WithoutChildrenOrChild<T> = Omit<T, 'children' | 'child'> & { children?: never; child?: never };

export function cn(...inputs: ClassValue[]): string {
  return twMerge(clsx(inputs));
}
