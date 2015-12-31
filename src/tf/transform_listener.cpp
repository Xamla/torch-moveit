#include "torch-tf.h"
#include <tf/transform_listener.h>

TFIMP(tf::TransformListener *, TransformListener, new)()
{
  return new tf::TransformListener();
}

TFIMP(void, TransformListener, delete)(tf::TransformListener *self)
{
  delete self;
}

TFIMP(void, TransformListener, clear)(tf::TransformListener *self)
{
  self->clear();
}

TFIMP(void, TransformListener, getFrameStrings)(tf::TransformListener *self, StringsPtr *result)
{
  self->getFrameStrings(**result);
}

TFIMP(void, TransformListener, lookupTransform)(tf::TransformListener *self, const char *target_frame, const char *source_frame, ros::Time *time, tf::StampedTransform *result)
{
  self->lookupTransform(target_frame, source_frame, *time, *result);
}

TFIMP(bool, TransformListener, waitForTransform)(tf::TransformListener *self, const char *target_frame, const char *source_frame, ros::Time *time, ros::Duration *timeout, StringsPtr *error_msg)
{
  if (error_msg && *error_msg)
    (*error_msg)->push_back(std::string());
  return self->waitForTransform(target_frame, source_frame, *time, *timeout, ros::Duration(0.01), error_msg && *error_msg ? &(*error_msg)->back() : NULL);
}


TFIMP(bool, TransformListener, canTransform)(tf::TransformListener *self, const char *target_frame, const char *source_frame, ros::Time *time, StringsPtr *error_msg)
{
  if (error_msg && *error_msg)
    (*error_msg)->push_back(std::string());
  return self->canTransform(target_frame, source_frame, *time, error_msg && *error_msg ? &(*error_msg)->back() : NULL);
}

TFIMP(void, TransformListener, resolve)(tf::TransformListener *self, const char *frame_name, StringsPtr *result)
{
  (*result)->push_back(self->resolve(frame_name));
}
