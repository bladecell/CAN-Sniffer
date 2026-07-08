<div class="card bg-base-200 shadow-xl max-w-md mx-auto mt-8">
  <div class="card-body">
    <h2 class="card-title">System Settings</h2>
    
    <div class="form-control w-full">
      <label class="label" for="baud">
        <span class="label-text">Baudrate</span>
      </label>
      <select id="baud" class="select select-bordered w-full">
        <option>500 kbps</option>
        <option>250 kbps</option>
      </select>
    </div>
    
    <div class="card-actions justify-end mt-4">
      <button class="btn btn-primary" onclick={() => alert("Saved!")}>Apply Changes</button>
    </div>
  </div>
</div>
