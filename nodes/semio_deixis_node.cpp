#include <ros/ros.h>

#include <semio_msgs_ros/DeicticRecognitionResult.h>
#include <semio_msgs_ros/DeicticTargets.h>

#include <semio/recognition/deictic_recognizer.h>
#include <semio/ros/humanoid_source_adapter.h>

class SemioDeixisNode
{
public:
    typedef semio_msgs_ros::DeicticRecognitionResult _DeicticRecognitionResultMsg;
    typedef semio_msgs_ros::DeicticRecognitionHumanoidItem _DeicticRecognitionHumanoidItemMsg;
    typedef semio_msgs_ros::DeicticRecognitionSourceItem _DeicticRecognitionSourceItemMsg;
    typedef semio_msgs_ros::DeicticRecognitionTopNItem _DeicticRecognitionTopNItemMsg;
    typedef semio_msgs_ros::DeicticTargets _DeicticTargetsMsg;
    typedef semio_msgs_ros::DeicticTarget _DeicticTargetMsg;

    ros::NodeHandle nh_rel_;
    ros::Publisher result_pub_;
    ros::Subscriber targets_sub_;

    semio::HumanoidSource::Ptr humanoid_source_ptr_;
    semio::DeicticRecognizer deictic_recognizer_;

    SemioDeixisNode( ros::NodeHandle & nh_rel, semio::HumanoidSource::Ptr humanoid_source_ptr )
    :
        nh_rel_( nh_rel ),
        result_pub_( nh_rel_.advertise<_DeicticRecognitionResultMsg>( "result", 100 ) ),
        targets_sub_( nh_rel_.subscribe( "targets", 10, &SemioDeixisNode::targetsCB, this ) ),
        humanoid_source_ptr_( humanoid_source_ptr )
    {
        //
    }

    void spin()
    {
        ros::Rate loop_rate( 30 );

        while( ros::ok() )
        {
            ros::spinOnce();

            deictic_recognizer_.getHumanoids() = humanoid_source_ptr_->update();
            semio::DeicticRecognitionResult const & result( deictic_recognizer_.calculateResult() );

            _DeicticRecognitionResultMsg result_msg;

            result_msg.humanoids.reserve( result.size() );

            for( auto const & humanoid_item : result )
            {
                semio::DeixisSourceMap const & sources( humanoid_item.second );

                _DeicticRecognitionHumanoidItemMsg humanoid_msg;
                humanoid_msg.id = static_cast<uint32_t>( humanoid_item.first );
                humanoid_msg.sources.reserve( sources.size() );

                for( auto const & source_item : sources )
                {
                    semio::DeixisTopNList const & top_n_list( source_item.second );

                    _DeicticRecognitionSourceItemMsg source_msg;
                    source_msg.name = source_item.first;
                    source_msg.top_n_list.reserve( top_n_list.size() );

                    for( auto const & list_item : top_n_list )
                    {
                        _DeicticRecognitionTopNItemMsg top_n_item_msg;
                        top_n_item_msg.likelihood = list_item.value_;
                        top_n_item_msg.target_name = list_item.data_;

                        source_msg.top_n_list.push_back( std::move( top_n_item_msg ) );
                    }

                    humanoid_msg.sources.push_back( std::move( source_msg ) );
                }

                result_msg.humanoids.push_back( std::move( humanoid_msg ) );
            }

            result_pub_.publish( std::move( result_msg ) );

            loop_rate.sleep();
        }
    }

    void targetsCB( _DeicticTargetsMsg::ConstPtr const & msg_ptr )
    {
        auto const & targets_msg = *msg_ptr;

        semio::DeicticTargetArray deictic_targets;

        for( auto const & target_msg : targets_msg.targets )
        {
            auto const & name = target_msg.name;
            auto const & position = target_msg.position;

            deictic_targets.emplace( name, Eigen::Vector3d( position.x, position.y, position.z ) );
        }

        deictic_recognizer_.getTargets() = deictic_targets;
    }
};

int main( int argc, char ** argv )
{
    ros::init( argc, argv, "semio_deixis_node" );
    ros::NodeHandle nh_rel( "~" );

    semio::ros::HumanoidSourceAdapter humanoid_source_adapter( nh_rel );

    SemioDeixisNode semio_deixis_node( nh_rel, humanoid_source_adapter.getHumanoidSource() );
    semio_deixis_node.spin();

    return 0;
}
