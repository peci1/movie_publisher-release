// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/**
 * \file
 * \brief Convert movie files and their metadata to ROS bag file.
 * \author Martin Pecka
 *
 * \par Stored topics
 *
 * - `${~topic}` (`sensor_msgs/Image`): The published movie (if raw transport is used).
 * - `${~topic}/${~transport}` (*): The published movie compressed stream (if raw transport is not used).
 * - `${~topic}/camera_info` (`sensor_msgs/CameraInfo`): Camera info.
 * - `${~topic}/azimuth` (`compass_msgs/Azimuth`): Georeferenced heading of the camera.
 * - `${~topic}/fix` (`sensor_msgs/NavSatFix`): GNSS position of the camera.
 * - `${~topic}/fix_detail` (`gps_common/GPSFix`): GNSS position of the camera.
 * - `${~topic}/imu` (`sensor_msgs/Imu`): Orientation and acceleration of the camera.
 *
 * To extract the additional topics except `movie`, the node uses instances of MetadataExtractor.
 *
 * To change the prefix of all topics, set `~topic` parameter. To change the name of a single topic, remap it.
 *
 * \par Parameters
 *
 * Parameters `~start`, `~end` and `~duration` can be expressed in seconds `(15.35)`, in `(min, sec)`,
 * in `(hour, min, sec)`, or as a string: `'01:03:05.35'`.
 *
 * - `~bag` (string, required): Path where the result should be stored.
 * - `~overwrite_bag` (bool, default false): If true and the bag file exists, it will be overwritten. Otherwise, it will
 *                                           be appended (and created if needed).
 * - `~movie` (string, required): Path to the movie to play. Any format that ffmpeg can decode.
 * - `~transport` (string, default `raw`, suggested `compressed`): The image_transport used to store the movie
 *                                                                 in the bag.
 * - `~start` (float|tuple|string, optional): If set, the movie will be read from the specified time.
 *                                            Cannot be set together with `~end` and `~duration`.
 * - `~end` (float|tuple|string, optional): If set, the movie will be read up to the specified time (not affected by
 *                                          start). Cannot be set together with `~start` and `~duration`.
 * - `~duration` (float|tuple|string, optional): If set, playback will have this duration. If end is also set, the
 *                                               duration is counted from the end of the clip, otherwise, it is the
 *                                               duration from the start of the clip. Cannot be set together with
 *                                               `~start` and `~end`.
 * - `~timestamp_offset` (int|float|string, default 0.0): Adjustment of timestamps determined by `~timestamp_source`.
 *                                                        If given as string, it can be a simple mathematical expression
 *                                                        that can also resolve several variables:
 *                                                        `ros_time` (current ROS time),
 *                                                        `wall_time` (current wall time),
 *                                                        `metadata_start` (start time from metadata),
 *                                                        `bag_start` (start time of the bag file),
 *                                                        `bag_end` (end time of the metadata),
 *                                                        `bag_duration` (duration of the bag file in s).
 * - `~timestamp_source` (str, default `metadata`): How to determine timestamps of the movie frames. Options are:
 *   - `metadata`: Extract absolute time when the movie was recorded and use that time as timestamps.
 *   - `all_zeros`: Use zero timestamps. Please note that time 0.0 cannot be stored in bag files. Use
 *                  `~timestamp_offset` to make the time valid.
 *   - `absolute_timecode`: Use the absolute timecode as timestamps (i.e. time since start of movie file).
 *   - `relative_timecode`: Use the relative timecode as timestamps (i.e. time since `~start`).
 *   - `ros_time`: Timestamp the frames with current ROS time. Note that this mode is not very useful for movie_to_bag.
 * - `~frame_id` (string, default ""): The frame_id used in the geometrical messages' headers.
 * - `~optical_frame_id` (string, default `${frame_id}_optical_frame`): The frame_id used in the image messages'
 *                                                                      headers.
 * - `~verbose` (bool, default False): If True, logs info about every frame played.
 * - `~allow_yuv_fallback` (bool, default False): Set whether `YUV***` formats should be decoded to YUV422, or whether
 *                                                the default encoding should be used.
 * - `~default_encoding` (string, optional): Set the default encoding which should be used for output frames if there is
 *                                           no direct match between the libav pixel format and ROS image encodings.
 * - `~encoding` (string, optional): Set the encoding which should be used for output frames regardless of their source
 *                                   encoding (one of sensor_msgs::image_encodings constants).
 */

#include <memory>
#include <string>

#include <cras_cpp_common/node_utils.hpp>
#include <cras_cpp_common/node_utils/node_with_optional_master.h>
#include <movie_publisher/movie_to_bag.h>


int main(int argc, char* argv[])
{
  const auto log = std::make_shared<cras::NodeLogHelper>();
  movie_publisher::MovieToBag node(log);

  // We're using NodeWithOptionalMaster, so this is instead of ros::init().
  const auto options = ros::init_options::AnonymousName;
  node.init(argc, argv, "movie_to_bag", options);

  const auto params = node.getPrivateParams();
  const auto bagFilename = params->getParam<std::string>("bag", cras::nullopt);
  const auto movieFilename = params->getParam<std::string>("movie", cras::nullopt);
  const auto transport = params->getParam<std::string>("transport", "raw");

  const auto openResult = node.open(bagFilename, transport, movieFilename, params);
  if (!openResult.has_value())
  {
    CRAS_ERROR("%s", openResult.error().c_str());
    return 1;
  }

  const auto runResult = node.run();
  if (!runResult.has_value())
  {
    CRAS_ERROR("%s", runResult.error().c_str());
    return 2;
  }

  return 0;
}
