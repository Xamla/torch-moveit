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

TFIMP(void, TransformListener, getFrameStrings)(tf::TransformListener *self, std::vector<std::string> *result)
{
  self->getFrameStrings(*result);
}

TFIMP(void, TransformListener, lookupTransform)(tf::TransformListener *self, const char *target_frame, const char *source_frame, ros::Time *time, tf::StampedTransform *result)
{
  self->lookupTransform(target_frame, source_frame, *time, *result);
}

TFIMP(bool, TransformListener, waitForTransform)(tf::TransformListener *self, const char *target_frame, const char *source_frame, ros::Time *time, ros::Duration *timeout, std::string *error_msg)
{
  return self->waitForTransform(target_frame, source_frame, *time, *timeout, ros::Duration(0.01), error_msg);
}

TFIMP(bool, TransformListener, canTransform)(tf::TransformListener *self, const char *target_frame, const char *source_frame, ros::Time *time, std::string *error_msg)
{
  return self->canTransform(target_frame, source_frame, *time, error_msg);
}

TFIMP(void, TransformListener, resolve)(tf::TransformListener *self, const char *frame_name, std::string *result)
{
  *result = self->resolve(frame_name);
}
