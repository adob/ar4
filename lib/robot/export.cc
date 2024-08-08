#include "export.h"

// #include <highfive/highfive.hpp>
#include "env.h"
#include "flatbuffer/recording_generated.h"

#include "tts/tts.h"

#include <flatbuffers/base.h>
#include <flatbuffers/buffer.h>
#include <flatbuffers/flatbuffer_builder.h>

#include "lib/print.h"

using namespace robot;

// static void mat2vec(cv::Mat const& mat, std::vector<uint8> &vec) {
//     if (!mat.isContinuous()) {
//         panic("not continuous");
//     }

//     vec.insert(vec.end(), mat.datastart, mat.dataend);
// }
// 2

static auto mat2vec(flatbuffers::FlatBufferBuilder &b, cv::Mat const& mat) {
    // cv::Mat m = mat;
    // if (!m.isContinuous()) {
    //     m = m.clone();
    // }
    // if (!m.isContinuous()) {
    //     panic("not continous");
    // }

    // usize len = usize(m.dataend - m.datastart);
    return b.CreateVector(std::vector<uint8>(mat));
}



void robot::export_episode(str path, std::list<Observation> const& obs, error &err) {
    print "export_episode", path;

    flatbuffers::FlatBufferBuilder b;
    std::vector<flatbuffers::Offset<flatbuffer::Observation>> fobs;

    for (Observation ob : obs) {
        // auto cam_side = mat2vec(b, ob.cam_side);
        // auto cam_wrist = mat2vec(b, ob.cam_wrist);

        flatbuffer::Pose target_pose(ob.target_pose.data);
        flatbuffer::Pose current_pose(ob.current_pose.data);
        
        std::vector<flatbuffers::Offset<flatbuffer::Image>> frames;
        for (cv::Mat &frame : ob.cams) {
            auto frame_data = mat2vec(b, frame);
            
            flatbuffer::ImageBuilder frame_builder(b);
            frame_builder.add_data(frame_data);
            frames.push_back(frame_builder.Finish());
        }

        auto cams = b.CreateVector(frames);
        flatbuffer::ObservationBuilder fobb(b);
        fobb.add_cameras(cams);
        
        // fobb.add_cam_side(cam_side);
        // fobb.add_cam_wrist(cam_wrist);
        // panic("implement cam");
        fobb.add_action(&target_pose);
        fobb.add_qpos(&current_pose);

        fobs.push_back(fobb.Finish());
    }

    

    auto frec = flatbuffer::CreateRecording(b, b.CreateVector(fobs));

    b.Finish(frec);
    os::write_file(path, str(b.GetBufferSpan()), err);
    

    // HighFive::File h5file(path.std_string(), HighFive::File::Create | HighFive::File::Truncate);
    // h5file.createAttribute("sim", false);

    // HighFive::Group observations = h5file.createGroup("observations");
    // HighFive::Group images = observations.createGroup("images");

    // usize num_steps = obs.size();
    // // usize vec_size = num_steps * 480 * 640 * 3;

    // std::vector<std::vector<uint8>> side_vecs, wrist_vecs;

    // std::vector<std::vector<float64>> actions, qpos;

    // for (Observation const& ob : obs) {
    //     side_vecs.push_back(mat2vec(ob.cam_side));
    //     wrist_vecs.push_back(mat2vec(ob.cam_wrist));
    //     qpos.push_back({ob.current_pose[0], ob.current_pose[1], ob.current_pose[2], ob.current_pose[3], ob.current_pose[4], ob.current_pose[5]});
    //     actions.push_back({ob.target_pose[0], ob.target_pose[1], ob.target_pose[2], ob.target_pose[3], ob.target_pose[4], ob.target_pose[5]});
    // }

    // images.createDataSet("cam_side", side_vecs);
    // images.createDataSet("cam_wrist", wrist_vecs);

    // HighFive::DataSet action_dataset = h5file.createDataSet<float64>("action", {num_steps, 6});
    // action_dataset.write(actions);

    // HighFive::DataSet qpos_dataset = observations.createDataSet<float64>("qpos", {num_steps, 6});
    // qpos_dataset.write(qpos);
    print "export episode DONE";
}
