[1mdiff --git a/src/thunderbots/software/ai/passing/pass_generator.cpp b/src/thunderbots/software/ai/passing/pass_generator.cpp[m
[1mindex c2ca25a..906f978 100644[m
[1m--- a/src/thunderbots/software/ai/passing/pass_generator.cpp[m
[1m+++ b/src/thunderbots/software/ai/passing/pass_generator.cpp[m
[36m@@ -180,9 +180,9 @@[m [mvoid PassGenerator::pruneAndReplacePasses()[m
     if (num_passes_to_keep_after_pruning.value() < num_passes_to_optimize.value() &&[m
         num_passes_to_keep_after_pruning.value() < passes_to_optimize.size())[m
     {[m
[31m-            passes_to_optimize.erase([m
[31m-                passes_to_optimize.begin() + num_passes_to_keep_after_pruning.value(),[m
[31m-                passes_to_optimize.end());[m
[32m+[m[32m        passes_to_optimize.erase([m
[32m+[m[32m            passes_to_optimize.begin() + num_passes_to_keep_after_pruning.value(),[m
[32m+[m[32m            passes_to_optimize.end());[m
     }[m
 [m
     // Generate new passes to replace the ones we just removed[m
[1mdiff --git a/src/thunderbots/software/ai/world/field.cpp b/src/thunderbots/software/ai/world/field.cpp[m
[1mindex 0a231d2..7c4e225 100644[m
[1m--- a/src/thunderbots/software/ai/world/field.cpp[m
[1m+++ b/src/thunderbots/software/ai/world/field.cpp[m
[36m@@ -1,12 +1,13 @@[m
 #include "field.h"[m
 [m
[32m+[m[32m#include "boost/circular_buffer.hpp"[m
 #include "geom/rectangle.h"[m
 #include "util/time/timestamp.h"[m
[31m-#include "boost/circular_buffer.hpp"[m
 [m
 Field::Field(double field_length, double field_width, double defense_length,[m
              double defense_width, double goal_width, double boundary_width,[m
[31m-             double center_circle_radius, const Timestamp& timestamp, unsigned int buffer_size)[m
[32m+[m[32m             double center_circle_radius, const Timestamp &timestamp,[m
[32m+[m[32m             unsigned int buffer_size)[m
     : field_length_(field_length),[m
       field_width_(field_width),[m
       defense_length_(defense_length),[m
[36m@@ -23,20 +24,20 @@[m [mField::Field(double field_length, double field_width, double defense_length,[m
 [m
 void Field::updateDimensions(const Field &new_field_data)[m
 {[m
[31m-    field_length_         = new_field_data.length();[m
[31m-    field_width_          = new_field_data.width();[m
[31m-    defense_width_        = new_field_data.defenseAreaWidth();[m
[31m-    defense_length_       = new_field_data.defenseAreaLength();[m
[31m-    goal_width_           = new_field_data.goalWidth();[m
[31m-    boundary_width_       = new_field_data.boundaryWidth();[m
[31m-    center_circle_radius_ = new_field_data.centreCircleRadius();[m
[32m+[m[32m    field_length_          = new_field_data.length();[m
[32m+[m[32m    field_width_           = new_field_data.width();[m
[32m+[m[32m    defense_width_         = new_field_data.defenseAreaWidth();[m
[32m+[m[32m    defense_length_        = new_field_data.defenseAreaLength();[m
[32m+[m[32m    goal_width_            = new_field_data.goalWidth();[m
[32m+[m[32m    boundary_width_        = new_field_data.boundaryWidth();[m
[32m+[m[32m    center_circle_radius_  = new_field_data.centreCircleRadius();[m
     last_update_timestamps = new_field_data.getTimestampHistory();[m
 }[m
 [m
 void Field::updateDimensions(double field_length, double field_width,[m
                              double defense_length, double defense_width,[m
                              double goal_width, double boundary_width,[m
[31m-                             double center_circle_radius, const Timestamp& timestamp)[m
[32m+[m[32m                             double center_circle_radius, const Timestamp &timestamp)[m
 {[m
     field_length_         = field_length;[m
     field_width_          = field_width;[m
[36m@@ -191,30 +192,34 @@[m [mbool Field::pointInFieldLines(const Point &p) const[m
     return fieldLines().containsPoint(p);[m
 }[m
 [m
[31m-boost::circular_buffer<Timestamp> Field::getTimestampHistory() const {[m
[32m+[m[32mboost::circular_buffer<Timestamp> Field::getTimestampHistory() const[m
[32m+[m[32m{[m
     return last_update_timestamps;[m
 }[m
 [m
[31m-Timestamp Field::getMostRecentTimestamp() const {[m
[32m+[m[32mTimestamp Field::getMostRecentTimestamp() const[m
[32m+[m[32m{[m
     return last_update_timestamps.front();[m
 }[m
 [m
[31m-void Field::updateTimestamp(Timestamp time_stamp) {[m
[31m-[m
[32m+[m[32mvoid Field::updateTimestamp(Timestamp time_stamp)[m
[32m+[m[32m{[m
     // Check if the timestamp buffer is empty[m
[31m-    if( last_update_timestamps.empty() ) {[m
[32m+[m[32m    if (last_update_timestamps.empty())[m
[32m+[m[32m    {[m
         last_update_timestamps.push_front(time_stamp);[m
     }[m
     // Check that the new timestamp is not older than the most recent timestamp[m
     else if (time_stamp < Field::getMostRecentTimestamp())[m
     {[m
         throw std::invalid_argument([m
[31m-                "Error: Attempt tp update Field state with old Timestamp");[m
[32m+[m[32m            "Error: Attempt tp update Field state with old Timestamp");[m
     }[m
[31m-    else {[m
[32m+[m[32m    else[m
[32m+[m[32m    {[m
         last_update_timestamps.push_front(time_stamp);[m
     }[m
[31m-    }[m
[32m+[m[32m}[m
 [m
 bool Field::operator==(const Field &other) const[m
 {[m
[1mdiff --git a/src/thunderbots/software/ai/world/field.h b/src/thunderbots/software/ai/world/field.h[m
[1mindex 2cbe340..71de133 100644[m
[1m--- a/src/thunderbots/software/ai/world/field.h[m
[1m+++ b/src/thunderbots/software/ai/world/field.h[m
[36m@@ -1,9 +1,9 @@[m
 #pragma once[m
 [m
[32m+[m[32m#include "boost/circular_buffer.hpp"[m
 #include "geom/point.h"[m
 #include "geom/rectangle.h"[m
 #include "util/time/timestamp.h"[m
[31m-#include "boost/circular_buffer.hpp"[m
 [m
 typedef enum[m
 {[m
[36m@@ -32,7 +32,8 @@[m [mclass Field[m
      */[m
     explicit Field(double field_length, double field_width, double defense_length,[m
                    double defense_width, double goal_width, double boundary_width,[m
[31m-                   double center_circle_radius, const Timestamp& timestamp, unsigned int buffer_size = 20);[m
[32m+[m[32m                   double center_circle_radius, const Timestamp &timestamp,[m
[32m+[m[32m                   unsigned int buffer_size = 20);[m
 [m
     /**[m
      * Updates the dimensions of the field. All units should be in metres.[m
[36m@@ -49,7 +50,7 @@[m [mclass Field[m
      */[m
     void updateDimensions(double field_length, double field_width, double defense_length,[m
                           double defense_width, double goal_width, double boundary_width,[m
[31m-                          double center_circle_radius, const Timestamp& timestamp);[m
[32m+[m[32m                          double center_circle_radius, const Timestamp &timestamp);[m
 [m
     /**[m
      * Updates the field with new data[m
[36m@@ -280,7 +281,8 @@[m [mclass Field[m
     /**[m
      * Returns the most Timestamp corresponding to the most recent update to Field object[m
      *[m
[31m-     * @return Timestamp : The Timestamp corresponding to the most recent update to the Field object[m
[32m+[m[32m     * @return Timestamp : The Timestamp corresponding to the most recent update to the[m
[32m+[m[32m     * Field object[m
      */[m
     Timestamp getMostRecentTimestamp() const;[m
 [m
[1mdiff --git a/src/thunderbots/software/network_input/backend.cpp b/src/thunderbots/software/network_input/backend.cpp[m
[1mindex a46171c..d065a36 100644[m
[1m--- a/src/thunderbots/software/network_input/backend.cpp[m
[1m+++ b/src/thunderbots/software/network_input/backend.cpp[m
[36m@@ -110,8 +110,9 @@[m [mField Backend::createFieldFromPacketGeometry([m
     double defense_width =[m
         (defense_width_p1 - defense_width_p2).len() * METERS_PER_MILLIMETER;[m
 [m
[31m-    Field field = Field(field_length, field_width, defense_length, defense_width,[m
[31m-                        goal_width, boundary_width, center_circle_radius, Timestamp::fromSeconds(0));[m
[32m+[m[32m    Field field =[m
[32m+[m[32m        Field(field_length, field_width, defense_length, defense_width, goal_width,[m
[32m+[m[32m              boundary_width, center_circle_radius, Timestamp::fromSeconds(0));[m
     return field;[m
 }[m
 [m
[1mdiff --git a/src/thunderbots/software/test/ai/world/field.cpp b/src/thunderbots/software/test/ai/world/field.cpp[m
[1mindex 48b99d0..da68fec 100644[m
[1m--- a/src/thunderbots/software/test/ai/world/field.cpp[m
[1m+++ b/src/thunderbots/software/test/ai/world/field.cpp[m
[36m@@ -48,7 +48,8 @@[m [mTEST_F(FieldTest, update_with_all_parameters)[m
     Field field_to_update = Field(0, 0, 0, 0, 0, 0, 0, Timestamp::fromSeconds(0));[m
 [m
     field_to_update.updateDimensions(length, width, defense_length, defense_width,[m
[31m-                                     goal_width, boundary_width, center_circle_radius, default_time_stamp);[m
[32m+[m[32m                                     goal_width, boundary_width, center_circle_radius,[m
[32m+[m[32m                                     default_time_stamp);[m
 [m
     EXPECT_DOUBLE_EQ(9.6, field_to_update.totalLength());[m
     EXPECT_DOUBLE_EQ(6.6, field_to_update.totalWidth());[m
[36m@@ -198,27 +199,35 @@[m [mTEST_F(FieldTest, equality_operator_fields_with_different_center_circle_radius)[m
 TEST_F(FieldTest, field_timestamp_history_is_saved)[m
 {[m
     Field field = Field(length, width, defense_length, defense_width, goal_width,[m
[31m-                          boundary_width, center_circle_radius, default_time_stamp);[m
[32m+[m[32m                        boundary_width, center_circle_radius, default_time_stamp);[m
 [m
     field.updateDimensions(length, width, defense_length, defense_width, goal_width,[m
[31m-                           boundary_width, center_circle_radius, Timestamp::fromSeconds(default_time_stamp.getSeconds()+1));[m
[32m+[m[32m                           boundary_width, center_circle_radius,[m
[32m+[m[32m                           Timestamp::fromSeconds(default_time_stamp.getSeconds() + 1));[m
 [m
     field.updateDimensions(length, width, defense_length, defense_width, goal_width,[m
[31m-                           boundary_width, center_circle_radius, Timestamp::fromSeconds(default_time_stamp.getSeconds()+2));[m
[31m-[m
[31m-    EXPECT_EQ( field.getTimestampHistory().size(), 3);[m
[31m-    EXPECT_EQ( field.getTimestampHistory()[0].getSeconds(), default_time_stamp.getSeconds()+2);[m
[31m-    EXPECT_EQ( field.getTimestampHistory()[1].getSeconds(), default_time_stamp.getSeconds()+1 );[m
[31m-    EXPECT_EQ( field.getTimestampHistory()[2].getSeconds(), default_time_stamp.getSeconds());[m
[31m-[m
[31m-    EXPECT_EQ( field.getMostRecentTimestamp().getSeconds(), default_time_stamp.getSeconds()+2);[m
[32m+[m[32m                           boundary_width, center_circle_radius,[m
[32m+[m[32m                           Timestamp::fromSeconds(default_time_stamp.getSeconds() + 2));[m
[32m+[m
[32m+[m[32m    EXPECT_EQ(field.getTimestampHistory().size(), 3);[m
[32m+[m[32m    EXPECT_EQ(field.getTimestampHistory()[0].getSeconds(),[m
[32m+[m[32m              default_time_stamp.getSeconds() + 2);[m
[32m+[m[32m    EXPECT_EQ(field.getTimestampHistory()[1].getSeconds(),[m
[32m+[m[32m              default_time_stamp.getSeconds() + 1);[m
[32m+[m[32m    EXPECT_EQ(field.getTimestampHistory()[2].getSeconds(),[m
[32m+[m[32m              default_time_stamp.getSeconds());[m
[32m+[m
[32m+[m[32m    EXPECT_EQ(field.getMostRecentTimestamp().getSeconds(),[m
[32m+[m[32m              default_time_stamp.getSeconds() + 2);[m
 }[m
 [m
 TEST_F(FieldTest, exception_thrown_when_older_timestamp_is_used)[m
 {[m
[31m-[m
[31m-    ASSERT_THROW(Field field = Field(length, width, defense_length, defense_width, goal_width,[m
[31m-                                     boundary_width, center_circle_radius, Timestamp::fromSeconds(default_time_stamp.getSeconds()-1)), std::invalid_argument);[m
[32m+[m[32m    ASSERT_THROW([m
[32m+[m[32m        Field field = Field(length, width, defense_length, defense_width, goal_width,[m
[32m+[m[32m                            boundary_width, center_circle_radius,[m
[32m+[m[32m                            Timestamp::fromSeconds(default_time_stamp.getSeconds() - 1)),[m
[32m+[m[32m        std::invalid_argument);[m
 }[m
 [m
 TEST_F(FieldTest, point_not_in_defense_area)[m
[1mdiff --git a/src/thunderbots/software/test/util/ros_messages.cpp b/src/thunderbots/software/test/util/ros_messages.cpp[m
[1mindex abf8d51..fbb0f33 100644[m
[1m--- a/src/thunderbots/software/test/util/ros_messages.cpp[m
[1m+++ b/src/thunderbots/software/test/util/ros_messages.cpp[m
[36m@@ -81,7 +81,7 @@[m [mTEST(ROSMessageUtilTest, create_field_from_ros_message)[m
     double defense_length       = 1.0;[m
     double boundary_width       = 0.3;[m
     double center_circle_radius = 0.5;[m
[31m-    double default_timestamp = 0;[m
[32m+[m[32m    double default_timestamp    = 0;[m
 [m
     thunderbots_msgs::Field field_msg;[m
 [m
[36m@@ -95,8 +95,9 @@[m [mTEST(ROSMessageUtilTest, create_field_from_ros_message)[m
 [m
     Field field = Util::ROSMessages::createFieldFromROSMessage(field_msg);[m
 [m
[31m-    Field field_other = Field(length, width, defense_length, defense_width, goal_width,[m
[31m-                              boundary_width, center_circle_radius, Timestamp::fromSeconds(default_timestamp));[m
[32m+[m[32m    Field field_other =[m
[32m+[m[32m        Field(length, width, defense_length, defense_width, goal_width, boundary_width,[m
[32m+[m[32m              center_circle_radius, Timestamp::fromSeconds(default_timestamp));[m
 [m
     EXPECT_EQ(field_other, field);[m
 }[m
[36m@@ -110,10 +111,11 @@[m [mTEST(ROSMessageUtilTest, convert_field_to_ros_message)[m
     double defense_length       = 1.0;[m
     double boundary_width       = 0.3;[m
     double center_circle_radius = 0.5;[m
[31m-    double default_timestamp = 0;[m
[32m+[m[32m    double default_timestamp    = 0;[m
 [m
[31m-    Field field = Field(length, width, defense_length, defense_width, goal_width,[m
[31m-                        boundary_width, center_circle_radius, Timestamp::fromSeconds(default_timestamp));[m
[32m+[m[32m    Field field =[m
[32m+[m[32m        Field(length, width, defense_length, defense_width, goal_width, boundary_width,[m
[32m+[m[32m              center_circle_radius, Timestamp::fromSeconds(default_timestamp));[m
 [m
     thunderbots_msgs::Field field_msg =[m
         Util::ROSMessages::convertFieldToROSMessage(field);[m
[1mdiff --git a/src/thunderbots/software/util/ros_messages.cpp b/src/thunderbots/software/util/ros_messages.cpp[m
[1mindex 64f791e..793c517 100644[m
[1m--- a/src/thunderbots/software/util/ros_messages.cpp[m
[1m+++ b/src/thunderbots/software/util/ros_messages.cpp[m
[36m@@ -70,7 +70,8 @@[m [mnamespace Util[m
             Field field = Field(field_msg.field_length, field_msg.field_width,[m
                                 field_msg.defense_length, field_msg.defense_width,[m
                                 field_msg.goal_width, field_msg.boundary_width,[m
[31m-                                field_msg.center_circle_radius, Timestamp::fromSeconds(field_msg.timestamp_seconds));[m
[32m+[m[32m                                field_msg.center_circle_radius,[m
[32m+[m[32m                                Timestamp::fromSeconds(field_msg.timestamp_seconds));[m
 [m
             return field;[m
         }[m
