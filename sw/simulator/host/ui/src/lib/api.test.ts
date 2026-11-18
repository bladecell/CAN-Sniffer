import { describe, it } from 'node:test';
import assert from 'node:assert/strict';
import { apiHeaders } from './api';

describe('simulator HTTP headers', () => {
  it('marks every HTTP mutation as an intentional UI action', () => {
    assert.deepEqual(apiHeaders('POST', true), { 'Content-Type': 'application/json', 'X-Simulator-Intent': 'ui' });
    assert.deepEqual(apiHeaders('DELETE', false), { 'X-Simulator-Intent': 'ui' });
  });

  it('leaves read requests without the mutation intent', () => {
    assert.deepEqual(apiHeaders('GET', false), {});
  });
});
