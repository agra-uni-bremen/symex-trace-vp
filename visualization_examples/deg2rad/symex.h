#pragma once

#include <stdint.h>
#include <stddef.h>

static volatile uint32_t* const SYMCTRL_ADDR = (uint32_t*)0x02020000;
static volatile uint32_t* const SYMCTRL_SIZE = (uint32_t*)0x02020004;
static volatile uint32_t* const SYMCTRL_CTRL = (uint32_t*)0x02020008;

#define SYMEX_ERROR (1 << 31);
#define SYMEX_EXIT  (1 << 30);

void
make_symbolic(void *ptr, size_t size)
{
	*SYMCTRL_ADDR = (uintptr_t)ptr;
	*SYMCTRL_SIZE = size;
}

void
symex_error(void)
{
	*SYMCTRL_CTRL = SYMEX_ERROR;
}

void
symex_exit(void)
{
	*SYMCTRL_CTRL = SYMEX_EXIT;
}

float guess_symbolic_signed_float() {
  float array[] = {
	-4.0f,
	-3.0f,
	-2.0f,
	-1.0f,
	0.0f,
	1.0f,
	2.0f,
	3.0f,
	4.0f,
  };
  size_t num_elements = sizeof(array) / sizeof(array[0]);
  size_t index;
  make_symbolic(&index, sizeof(index));
  index = index % num_elements;
  for (size_t i = 0; i < num_elements; i++) {
	if (index == i) {
	  return array[i];
	}
  }
  symex_exit();
  return 0.0f;
}

float guess_symbolic_unsigned_float() {
  float array[] = {
	0.0f,
	1.0f,
	2.0f,
	3.0f
  };
  size_t num_elements = sizeof(array) / sizeof(array[0]);
  size_t index;
  make_symbolic(&index, sizeof(index));
  index = index % num_elements;
  for (size_t i = 0; i < num_elements; i++) {
	if (index == i) {
	  return array[i];
	}
  }
  symex_exit();
  return 0.0f;
}
