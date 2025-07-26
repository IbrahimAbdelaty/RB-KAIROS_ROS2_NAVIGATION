// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from robotnik_io_msgs:srv/SetAnalogOutputArray.idl
// generated code does not contain a copyright notice
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <Python.h>
#include <stdbool.h>
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-function"
#endif
#include "numpy/ndarrayobject.h"
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif
#include "rosidl_runtime_c/visibility_control.h"
#include "robotnik_io_msgs/srv/detail/set_analog_output_array__struct.h"
#include "robotnik_io_msgs/srv/detail/set_analog_output_array__functions.h"

#include "rosidl_runtime_c/primitives_sequence.h"
#include "rosidl_runtime_c/primitives_sequence_functions.h"

// Nested array functions includes
#include "robotnik_io_msgs/msg/detail/analog_io__functions.h"
// end nested array functions include
bool robotnik_io_msgs__msg__analog_io__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * robotnik_io_msgs__msg__analog_io__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool robotnik_io_msgs__srv__set_analog_output_array__request__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[75];
    {
      char * class_name = NULL;
      char * module_name = NULL;
      {
        PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
        if (class_attr) {
          PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
          if (name_attr) {
            class_name = (char *)PyUnicode_1BYTE_DATA(name_attr);
            Py_DECREF(name_attr);
          }
          PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
          if (module_attr) {
            module_name = (char *)PyUnicode_1BYTE_DATA(module_attr);
            Py_DECREF(module_attr);
          }
          Py_DECREF(class_attr);
        }
      }
      if (!class_name || !module_name) {
        return false;
      }
      snprintf(full_classname_dest, sizeof(full_classname_dest), "%s.%s", module_name, class_name);
    }
    assert(strncmp("robotnik_io_msgs.srv._set_analog_output_array.SetAnalogOutputArray_Request", full_classname_dest, 74) == 0);
  }
  robotnik_io_msgs__srv__SetAnalogOutputArray_Request * ros_message = _ros_message;
  {  // output
    PyObject * field = PyObject_GetAttrString(_pymsg, "output");
    if (!field) {
      return false;
    }
    PyObject * seq_field = PySequence_Fast(field, "expected a sequence in 'output'");
    if (!seq_field) {
      Py_DECREF(field);
      return false;
    }
    Py_ssize_t size = PySequence_Size(field);
    if (-1 == size) {
      Py_DECREF(seq_field);
      Py_DECREF(field);
      return false;
    }
    if (!robotnik_io_msgs__msg__AnalogIO__Sequence__init(&(ros_message->output), size)) {
      PyErr_SetString(PyExc_RuntimeError, "unable to create robotnik_io_msgs__msg__AnalogIO__Sequence ros_message");
      Py_DECREF(seq_field);
      Py_DECREF(field);
      return false;
    }
    robotnik_io_msgs__msg__AnalogIO * dest = ros_message->output.data;
    for (Py_ssize_t i = 0; i < size; ++i) {
      if (!robotnik_io_msgs__msg__analog_io__convert_from_py(PySequence_Fast_GET_ITEM(seq_field, i), &dest[i])) {
        Py_DECREF(seq_field);
        Py_DECREF(field);
        return false;
      }
    }
    Py_DECREF(seq_field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * robotnik_io_msgs__srv__set_analog_output_array__request__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of SetAnalogOutputArray_Request */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("robotnik_io_msgs.srv._set_analog_output_array");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "SetAnalogOutputArray_Request");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  robotnik_io_msgs__srv__SetAnalogOutputArray_Request * ros_message = (robotnik_io_msgs__srv__SetAnalogOutputArray_Request *)raw_ros_message;
  {  // output
    PyObject * field = NULL;
    size_t size = ros_message->output.size;
    field = PyList_New(size);
    if (!field) {
      return NULL;
    }
    robotnik_io_msgs__msg__AnalogIO * item;
    for (size_t i = 0; i < size; ++i) {
      item = &(ros_message->output.data[i]);
      PyObject * pyitem = robotnik_io_msgs__msg__analog_io__convert_to_py(item);
      if (!pyitem) {
        Py_DECREF(field);
        return NULL;
      }
      int rc = PyList_SetItem(field, i, pyitem);
      (void)rc;
      assert(rc == 0);
    }
    assert(PySequence_Check(field));
    {
      int rc = PyObject_SetAttrString(_pymessage, "output", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}

#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
// already included above
// #include <Python.h>
// already included above
// #include <stdbool.h>
// already included above
// #include "numpy/ndarrayobject.h"
// already included above
// #include "rosidl_runtime_c/visibility_control.h"
// already included above
// #include "robotnik_io_msgs/srv/detail/set_analog_output_array__struct.h"
// already included above
// #include "robotnik_io_msgs/srv/detail/set_analog_output_array__functions.h"

ROSIDL_GENERATOR_C_IMPORT
bool robotnik_common_msgs__msg__response__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * robotnik_common_msgs__msg__response__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool robotnik_io_msgs__srv__set_analog_output_array__response__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[76];
    {
      char * class_name = NULL;
      char * module_name = NULL;
      {
        PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
        if (class_attr) {
          PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
          if (name_attr) {
            class_name = (char *)PyUnicode_1BYTE_DATA(name_attr);
            Py_DECREF(name_attr);
          }
          PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
          if (module_attr) {
            module_name = (char *)PyUnicode_1BYTE_DATA(module_attr);
            Py_DECREF(module_attr);
          }
          Py_DECREF(class_attr);
        }
      }
      if (!class_name || !module_name) {
        return false;
      }
      snprintf(full_classname_dest, sizeof(full_classname_dest), "%s.%s", module_name, class_name);
    }
    assert(strncmp("robotnik_io_msgs.srv._set_analog_output_array.SetAnalogOutputArray_Response", full_classname_dest, 75) == 0);
  }
  robotnik_io_msgs__srv__SetAnalogOutputArray_Response * ros_message = _ros_message;
  {  // response
    PyObject * field = PyObject_GetAttrString(_pymsg, "response");
    if (!field) {
      return false;
    }
    if (!robotnik_common_msgs__msg__response__convert_from_py(field, &ros_message->response)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * robotnik_io_msgs__srv__set_analog_output_array__response__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of SetAnalogOutputArray_Response */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("robotnik_io_msgs.srv._set_analog_output_array");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "SetAnalogOutputArray_Response");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  robotnik_io_msgs__srv__SetAnalogOutputArray_Response * ros_message = (robotnik_io_msgs__srv__SetAnalogOutputArray_Response *)raw_ros_message;
  {  // response
    PyObject * field = NULL;
    field = robotnik_common_msgs__msg__response__convert_to_py(&ros_message->response);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "response", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
