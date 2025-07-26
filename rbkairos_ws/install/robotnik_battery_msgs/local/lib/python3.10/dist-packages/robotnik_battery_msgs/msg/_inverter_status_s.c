// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from robotnik_battery_msgs:msg/InverterStatus.idl
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
#include "robotnik_battery_msgs/msg/detail/inverter_status__struct.h"
#include "robotnik_battery_msgs/msg/detail/inverter_status__functions.h"

#include "rosidl_runtime_c/string.h"
#include "rosidl_runtime_c/string_functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool robotnik_battery_msgs__msg__inverter_status__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[58];
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
    assert(strncmp("robotnik_battery_msgs.msg._inverter_status.InverterStatus", full_classname_dest, 57) == 0);
  }
  robotnik_battery_msgs__msg__InverterStatus * ros_message = _ros_message;
  {  // ac_voltage
    PyObject * field = PyObject_GetAttrString(_pymsg, "ac_voltage");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->ac_voltage = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // dc_voltage
    PyObject * field = PyObject_GetAttrString(_pymsg, "dc_voltage");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->dc_voltage = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // load
    PyObject * field = PyObject_GetAttrString(_pymsg, "load");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->load = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // percentage
    PyObject * field = PyObject_GetAttrString(_pymsg, "percentage");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->percentage = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // temperature
    PyObject * field = PyObject_GetAttrString(_pymsg, "temperature");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->temperature = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // on
    PyObject * field = PyObject_GetAttrString(_pymsg, "on");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->on = (Py_True == field);
    Py_DECREF(field);
  }
  {  // serial_number
    PyObject * field = PyObject_GetAttrString(_pymsg, "serial_number");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->serial_number, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * robotnik_battery_msgs__msg__inverter_status__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of InverterStatus */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("robotnik_battery_msgs.msg._inverter_status");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "InverterStatus");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  robotnik_battery_msgs__msg__InverterStatus * ros_message = (robotnik_battery_msgs__msg__InverterStatus *)raw_ros_message;
  {  // ac_voltage
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->ac_voltage);
    {
      int rc = PyObject_SetAttrString(_pymessage, "ac_voltage", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // dc_voltage
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->dc_voltage);
    {
      int rc = PyObject_SetAttrString(_pymessage, "dc_voltage", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // load
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->load);
    {
      int rc = PyObject_SetAttrString(_pymessage, "load", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // percentage
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->percentage);
    {
      int rc = PyObject_SetAttrString(_pymessage, "percentage", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // temperature
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->temperature);
    {
      int rc = PyObject_SetAttrString(_pymessage, "temperature", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // on
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->on ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "on", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // serial_number
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->serial_number.data,
      strlen(ros_message->serial_number.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "serial_number", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
