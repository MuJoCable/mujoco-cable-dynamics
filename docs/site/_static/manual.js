document.addEventListener("DOMContentLoaded", () => {
  const field = document.querySelector("[data-example-filter]");
  if (!field) return;
  const items = [...document.querySelectorAll(".example-item")];
  const count = document.querySelector("[data-example-count]");
  field.addEventListener("input", () => {
    const query = field.value.trim().toLocaleLowerCase();
    for (const item of items) item.hidden = !item.textContent.toLocaleLowerCase().includes(query);
    if (count) count.textContent = `${items.filter(item => !item.hidden).length} / ${items.length}`;
  });
});
