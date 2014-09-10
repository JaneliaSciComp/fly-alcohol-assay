#ifndef _ARRAY_H_
#define _ARRAY_H_
#include <string.h>

template <class T, int size> class Array {
 public:
  Array();
  Array(T value);
  Array(T values[size]);
  T& operator[](int i);

 private:
  T _values[size];
};

template <class T, int size>
  Array<T,size>::Array() {}

template <class T, int size>
  Array<T,size>::Array(T value) {
  for (int i=0; i<size; i++) {
    _values[i] = value;
  }
}

template <class T, int size>
  Array<T, size>::Array(T values[size]) {
  memcpy((void*) _values, (void*) values, size*sizeof(T));
}

template <class T, int size>
  T& Array<T, size>::operator[](int i) {
  if ((i>=0) && (i<size)) {
    return _values[i];
  }
  else {
    T dummy;
    return dummy;
  }
}

#endif
