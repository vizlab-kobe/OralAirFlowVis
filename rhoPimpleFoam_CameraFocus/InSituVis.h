#pragma once
#include <random>
#include <kvs/OrthoSlice>
#include <kvs/Isosurface>
#include <kvs/PolygonObject>
#include <kvs/PolygonRenderer>
#include <kvs/PolygonImporter>
#include <kvs/UnstructuredVolumeObject>
#include <kvs/StochasticLineRenderer>
#include <kvs/StochasticPolygonRenderer>
#include <kvs/ParticleBasedRenderer>
#include <kvs/CellByCellMetropolisSampling>
#include <kvs/ExternalFaces>
#include <kvs/Bounds>
#include <kvs/String>
#include <kvs/StampTimer>
#include <kvs/StampTimerList>
#include <kvs/Math>
#include <kvs/mpi/StampTimer>
#include <InSituVis/Lib/Adaptor.h>
#include <InSituVis/Lib/Viewpoint.h>
#include <InSituVis/Lib/CubicViewpoint.h>
#include <InSituVis/Lib/SphericalViewpoint.h>
#include <InSituVis/Lib/PolyhedralViewpoint.h>
#include <InSituVis/Lib/TimestepControlledAdaptor.h>
#include <InSituVis/Lib/StochasticRenderingAdaptor.h>
#include <InSituVis/Lib/CFCA.h>
#include <InSituVis/Lib/CameraFocusControlledAdaptor_mpi.h>

/*****************************************************************************/
// In-situ visualization settings
/*****************************************************************************/

// Adaptor setting
//----------------------------------------------------------------------------
#define IN_SITU_VIS__ADAPTOR__CAMERA_CONTROL
//#define IN_SITU_VIS__ADAPTOR__CFCA

// Pipeline setting
//----------------------------------------------------------------------------
//#define IN_SITU_VIS__PIPELINE__ORTHO_SLICE
#define IN_SITU_VIS__PIPELINE__ISOSURFACE
//#define IN_SITU_VIS__PIPELINE__EXTERNAL_FACE
//#define IN_SITU_VIS__PIPELINE__WHOLE_MIN_MAX_VALUES

// Viewpoint setting
//----------------------------------------------------------------------------
#define IN_SITU_VIS__VIEWPOINT__SINGLE
//#define IN_SITU_VIS__VIEWPOINT__MULTIPLE_SPHERICAL
//#define IN_SITU_VIS__VIEWPOINT__MULTIPLE_POLYHEDRAL

// Adaptor definition
//----------------------------------------------------------------------------
#if defined( IN_SITU_VIS__ADAPTOR__CAMERA_CONTROL )
namespace { using Adaptor = InSituVis::mpi::CameraFocusControlledAdaptor; }
#elif defined( IN_SITU_VIS__ADAPTOR__CFCA )
namespace { using Adaptor =  InSituVis::mpi::CFCA; }
#else
namespace { using Adaptor = InSituVis::mpi::Adaptor; }
#endif

// Viewpoint loc
//----------------------------------------------------------------------------
const auto Pos = [] ( const float r )
{
    const auto tht = kvs::Math::pi / 4.0f;
    const auto phi = kvs::Math::pi / 4.0f;
    const auto x = r * std::sin( tht ) * std::sin( phi );
    const auto y = r * std::cos( tht );
    const auto z = r * std::sin( tht ) * std::cos( phi );
    return kvs::Vec3{ x, y, z };
};

// Parameters
//----------------------------------------------------------------------------
namespace Params
{
struct Output
{
    static const auto Image = true;
    static const auto SubImage = false;
    static const auto SubImageDepth = false;
    static const auto SubImageAlpha = false;
    static const auto Entropies = true;
    static const auto FrameEntropies = true;
    static const auto ZoomEntropies = true;
};
const auto EstimateIncludingBox = false;
const auto kotei = false;
const auto VisibleBoundingBox = true;
const auto VisibleBoundaryMesh = false;
const auto AutoZoom = true;

const auto ImageSize = kvs::Vec2ui{ 512, 512 }; // width x height
const auto AnalysisInterval = 10; // l: analysis (visuaization) time interval

// For IN_SITU_VIS__VIEWPOINT__*
//const auto ViewPos = kvs::Vec3{-6.0f,-2.0f,4.0f}; // viewpoint position
const auto ViewPos = kvs::Vec3{0.0f,0.0f,12.0f}; // viewpoint position

const auto ViewDim = kvs::Vec3ui{ 1, 5, 10 }; // viewpoint dimension
//const auto ViewDim = kvs::Vec3ui{ 1, 35, 70 }; // viewpoint dimension
const auto ViewDir = InSituVis::Viewpoint::Direction::Uni; // Uni or Omni
//add

//const auto Viewpoint = InSituVis::Viewpoint{ { 000000, ViewDir, ViewPos , kvs::Vec3{0,1,0}, rotation} };
const auto Viewpoint = InSituVis::Viewpoint{ { ViewDir, ViewPos } };
const auto ViewpointSpherical = InSituVis::SphericalViewpoint{ ViewDim, ViewDir };
const auto ViewpointPolyhedral = InSituVis::PolyhedralViewpoint{ ViewDim, ViewDir };

// For IN_SITU_VIS__ADAPTOR__CAMERA_FOCUS_CONTROLL
const auto ZoomLevel = 5;
const auto FrameDivs = kvs::Vec2ui{ 20, 20 };
const auto EntropyInterval = 1; // L: entropy calculation time interval
const auto MixedRatio = 0.5f; // mixed entropy ratio
auto LightEnt = ::Adaptor::LightnessEntropy();
auto DepthEnt = ::Adaptor::DepthEntropy();
auto MixedEnt = ::Adaptor::MixedEntropy( LightEnt, DepthEnt, MixedRatio );

// Entropy function
auto EntropyFunction = MixedEnt;
//auto EntropyFunction = LightEnt;
//auto EntropyFunction = DepthEnt;

// Path interpolator
auto Interpolator = ::Adaptor::Squad();
//auto Interpolator = ::Adaptor::Slerp();

// For IN_SITU_VIS__ADAPTOR__STOCHASTIC_RENDERING
const auto Repeats = 50; // number of repetitions for stochastic rendering
const auto BoundaryMeshOpacity = 30; // opacity value [0-255] of boundary mesh
}

/*****************************************************************************/




namespace local
{

class InSituVis : public ::Adaptor
{
    using BaseClass = ::Adaptor;
    using Object = BaseClass::Object;
    using Volume = kvs::UnstructuredVolumeObject;
    using Screen = BaseClass::Screen;

public:
    static Pipeline WholeMinMaxValues();
    static Pipeline OrthoSlice();
    static Pipeline Isosurface();
    static Pipeline ExternalFace( const kvs::mpi::Communicator& world );
    static Pipeline StochasticRendering( const size_t repeats );

private:
    kvs::PolygonObject m_boundary_mesh; ///< boundary mesh
    kvs::mpi::StampTimer m_sim_timer{ BaseClass::world() }; ///< timer for sim. process
    kvs::mpi::StampTimer m_cnv_timer{ BaseClass::world() }; ///< timer for cnv. process
    kvs::mpi::StampTimer m_vis_timer{ BaseClass::world() }; ///< timer for vis. process
    kvs::Real64 m_whole_min_value = 0.0; ///< min. value of whole time-varying volume data
    kvs::Real64 m_whole_max_value = 0.0; ///< max. value of whole time-varying volume data

public:
    InSituVis( const MPI_Comm world = MPI_COMM_WORLD, const int root = 0 ): BaseClass( world, root )
    {
        // Common parameters.
        this->setImageSize( Params::ImageSize.x(), Params::ImageSize.y() );
        this->setOutputImageEnabled( Params::Output::Image );
        this->setOutputSubImageEnabled(
            Params::Output::SubImage,
            Params::Output::SubImageDepth,
            Params::Output::SubImageAlpha );

        this->setOutputEntropiesEnabled( Params::Output::Entropies );
        this->setOutputFrameEntropiesEnabled( Params::Output::FrameEntropies );

        // Import boundary mesh.
        this->importBoundaryMesh( "./constant/triSurface/realistic-cfd3.stl" );

        // Time intervals.
        this->setAnalysisInterval( Params::AnalysisInterval );

#if defined( IN_SITU_VIS__ADAPTOR__CAMERA_CONTROL )
        this->setZoomLevel( Params::ZoomLevel );
        this->setFrameDivisions( Params::FrameDivs );
        this->setEntropyInterval( Params::EntropyInterval );
        this->setEntropyFunction( Params::EntropyFunction );
        this->setInterpolator( Params::Interpolator );
        this->setOutputFrameEntropiesEnabled( Params::Output::FrameEntropies );
        this->setAutoZoomingEnabled( Params::AutoZoom );

#endif

        // Set visualization pipeline.
#if defined( IN_SITU_VIS__ADAPTOR__STOCHASTIC_RENDERING )
        this->setRepetitionLevel( Params::Repeats );
        this->setPipeline( local::InSituVis::StochasticRendering( Params::Repeats ) );
#elif defined( IN_SITU_VIS__PIPELINE__ORTHO_SLICE )
        this->setPipeline( local::InSituVis::OrthoSlice() );
#elif defined( IN_SITU_VIS__PIPELINE__ISOSURFACE )
        this->setPipeline( local::InSituVis::Isosurface() );
#elif defined( IN_SITU_VIS__PIPELINE__EXTERNAL_FACE )
        this->setPipeline( local::InSituVis::ExternalFace( BaseClass::world() ) );
#endif

        // Set viewpoint(s)
#if defined( IN_SITU_VIS__VIEWPOINT__SINGLE )
        this->setViewpoint( Params::Viewpoint );
#elif defined( IN_SITU_VIS__VIEWPOINT__MULTIPLE_SPHERICAL )
        this->setViewpoint( Params::ViewpointSpherical );
#elif defined( IN_SITU_VIS__VIEWPOINT__MULTIPLE_POLYHEDRAL )
        this->setViewpoint( Params::ViewpointPolyhedral );
#endif
    }

    kvs::mpi::StampTimer& simTimer() { return m_sim_timer; }
    kvs::mpi::StampTimer& cnvTimer() { return m_cnv_timer; }
    kvs::mpi::StampTimer& visTimer() { return m_vis_timer; }

    void exec( const BaseClass::SimTime sim_time )
    {
        if ( !BaseClass::screen().scene()->hasObject( "BoundaryMesh") )
        {
            const bool visible = BaseClass::world().isRoot();

            // Boundary mesh
            auto* mesh = new kvs::PolygonObject();
            mesh->shallowCopy( m_boundary_mesh );
            mesh->setName( "BoundaryMesh" );
            mesh->setVisible( visible && Params::VisibleBoundaryMesh );

            // Bounding box
            kvs::Bounds bounds( kvs::RGBColor::Black(), 1.0f );
            auto* bbox = bounds.outputLineObject( mesh );
            bbox->setName( "BoundingBox" );
            bbox->setVisible( visible && Params::VisibleBoundingBox );

            // Register the bounding box at the root rank.
#if defined( IN_SITU_VIS__ADAPTOR__STOCHASTIC_RENDERING )
            object->setOpacity( 30 );

            auto* bbox_rendererr = new kvs::StochasticLineRenderer();
            auto* mesh_renderer = new kvs::StochasticPolygonRenderer();
            mesh_renderer->setTwoSideLightingEnabled( true );
            mesh_renderer->setEdgeFactor( 0.6f );

            BaseClass::screen().registerObject( bbox, bbox_renderer );
            BaseClass::screen().registerObject( mesh, mesh_renderer );
#else
            auto* mesh_renderer = new kvs::glsl::PolygonRenderer();
            mesh_renderer->setTwoSideLightingEnabled( true );
            mesh_renderer->setShadingModel( kvs::Shader::Lambert() );
            BaseClass::screen().registerObject( bbox );
            BaseClass::screen().registerObject( mesh, mesh_renderer );
#endif
        }

#if defined( IN_SITU_VIS__UPDATE_MIN_MAX_VALUES )
        // Update min/max values of the volume data in each time step.
        auto min_value = Volume::DownCast( *BaseClass::objects().begin() )->minValue();
        auto max_value = Volume::DownCast( *BaseClass::objects().begin() )->maxValue();
        for ( auto& object : BaseClass::objects() )
        {
            auto* volume = Volume::DownCast( object.get() );
            volume->updateMinMaxValues();

            min_value = kvs::Math::Min( min_value, volume->minValue() );
            max_value = kvs::Math::Max( max_value, volume->maxValue() );
        }

        BaseClass::world().allReduce( min_value, min_value, MPI_MIN );
        BaseClass::world().allReduce( max_value, max_value, MPI_MAX );

        for ( auto& object : BaseClass::objects() )
        {
            auto* volume = Volume::DownCast( object.get() );
            volume->setMinMaxValues( min_value, max_value );
        }
#endif

#if defined( IN_SITU_VIS__CALCULATE_WHOLE_MIN_MAX_VALUES )
        auto min_value = kvs::Value<kvs::Real64>::Max();
        auto max_value = kvs::Value<kvs::Real64>::Min();
        for ( auto& object : BaseClass::objects() )
        {
            auto* volume = Volume::DownCast( object.get() );
            volume->updateMinMaxValues();

            min_value = kvs::Math::Min( min_value, volume->minValue() );
            max_value = kvs::Math::Max( max_value, volume->maxValue() );
        }

        BaseClass::world().allReduce( min_value, min_value, MPI_MIN );
        BaseClass::world().allReduce( max_value, max_value, MPI_MAX );

        for ( auto& object : BaseClass::objects() )
        {
            auto* volume = Volume::DownCast( object.get() );
            volume->setMinMaxValues( min_value, max_value );
        }

        log() << "Min: " << min_value << std::endl;
        log() << "Max: " << max_value << std::endl;

        if ( sim_time.index == 1 )
        {
            m_whole_min_value = min_value;
            m_whole_max_value = max_value;
        }
        else
        {
            m_whole_min_value = kvs::Math::Min( m_whole_min_value, min_value );
            m_whole_max_value = kvs::Math::Max( m_whole_max_value, max_value );
        }

        log() << "Whole Min: " << m_whole_min_value << std::endl;
        log() << "Whole Max: " << m_whole_max_value << std::endl;
#endif
BaseClass::exec( sim_time );
}


#if defined( IN_SITU_VIS__ADAPTOR__CAMERA_CONTROL )
    void execRendering()
    {
        if ( !Params::VisibleBoundaryMesh && !Params::VisibleBoundingBox )
        {   
            BaseClass::execRendering();
            return;
        }

        auto* mesh = kvs::PolygonObject::DownCast( BaseClass::screen().scene()->object( "BoundaryMesh" ) );
        if ( mesh && Params::VisibleBoundaryMesh ) { mesh->setVisible( false ); }

        auto* bbox = kvs::LineObject::DownCast( BaseClass::screen().scene()->object( "BoundingBox" ) );
        if ( bbox && Params::VisibleBoundingBox ) { bbox->setVisible( false ); }

      if (Params::EstimateIncludingBox == false)
        {   
            
        BaseClass::execRendering();
        const bool visible = BaseClass::world().isRoot();
        if ( mesh ) { mesh->setVisible( visible && Params::VisibleBoundaryMesh ); }
        if ( bbox ) { bbox->setVisible( visible && Params::VisibleBoundingBox ); }

            if ( BaseClass::isEntropyStep() )
            {
                const auto index = BaseClass::maxIndex();
                const auto focus = BaseClass::maxFocusPoint();
                auto location = BaseClass::focusedLocation( BaseClass::viewpoint().at( index ), focus );
                const auto zoom_level = BaseClass::zoomLevel();
                auto p = location.position;
                for ( size_t level = 0; level < zoom_level; level++ )
                {
                    // Update camera position.
                    auto t = static_cast<float>( level ) / static_cast<float>( zoom_level );
                    location.position = ( 1 - t ) * p + t * focus;
                    if(Params::kotei == true)
                    {
                       location.look_at = {0,0,0};
                       location.up_vector = {0,1,0};
                    }
                    // Output the rendering images and the heatmap of entropies.
                        if ( Params::AutoZoom == false )
                        {
                            auto frame_buffer =  BaseClass::readback( location );
                            if ( BaseClass::world().isRoot() )
                            { 
                                if ( BaseClass::isOutputImageEnabled() )
                                {
                                    this->outputColorImage( location, frame_buffer, level );
                                }
                            }
                        }
                }
                if ( Params::AutoZoom )
                {   
                    location = BaseClass::viewpoint().at( index );
                    auto bestlocation = BaseClass::focusedLocation( location , focus ); 
                    bestlocation.position = BaseClass::estimatedZoomPosition();
                    bestlocation.rotation = BaseClass::maxRotation();
                    const auto level = BaseClass::estimatedZoomLevel();
                    auto frame_buffer = BaseClass::readback( bestlocation );
                    if ( BaseClass::world().isRoot() )
                    {
                        if ( BaseClass::isOutputImageEnabled() )
                        {
                            BaseClass::outputColorImage( bestlocation, frame_buffer, level );
                        }
                    }
                }
        
            }
            else
            {
                const auto focus = BaseClass::maxFocusPoint();
                auto location = BaseClass::erpLocation( focus );
                const auto zoom_level = BaseClass::zoomLevel();
                const auto p = location.position;
                for ( size_t level = 0; level < zoom_level; level++ )
                {
                    auto t = static_cast<float>( level ) / static_cast<float>( zoom_level );
                    location.position = ( 1 - t ) * p + t * focus;
                    //注視点を固定させたい時
                    if(Params::kotei)
                    {
                        location.look_at = {0,0,0};
                        location.up_vector = {0,1,0};
                    }
                    if ( Params::AutoZoom == false )
                    {
                        auto frame_buffer = BaseClass::readback( location );
                        // Output the rendering images and the heatmap of entropies.
                        if ( BaseClass::world().isRoot() )
                        {
                            if ( BaseClass::isOutputImageEnabled() )
                            {   
                                BaseClass::outputColorImage( location, frame_buffer, level );
                                //BaseClass::outputDepthImage( location, frame_buffer, level );
                            }
                        }
                    }
                }
                if(Params::AutoZoom)
                {
                    const auto focus = BaseClass::erpFocus();
                    auto bestlocation = BaseClass::erpLocation( focus );
                    //location.position = BaseClass::bestLocationPosition();
                    //auto bestlocation = BaseClass::erpLocation( focus );
                    //auto location = BaseClass::bestLocation();
                    const auto level = BaseClass::estimatedZoomLevel();
                    auto frame_buffer = BaseClass::readback( bestlocation );

                    // Output the rendering images and the heatmap of entropies.
                    if ( BaseClass::world().isRoot() )
                    {
                        if ( BaseClass::isOutputImageEnabled() )
                        {  
                            BaseClass::outputColorImage( bestlocation, frame_buffer, level );
                        }
                    }
                }
            }
        }
        else
        {
            const bool visible = BaseClass::world().isRoot();
            if ( mesh ) { mesh->setVisible( visible && Params::VisibleBoundaryMesh ); }
            if ( bbox ) { bbox->setVisible( visible && Params::VisibleBoundingBox ); }
            BaseClass::execRendering();
        }
    }
#elif defined( IN_SITU_VIS__ADAPTOR__CFCA )
    void execRendering()
    {
        if ( !Params::VisibleBoundaryMesh && !Params::VisibleBoundingBox )
        {
            BaseClass::execRendering();
            return;
        }

        auto* bbox = kvs::LineObject::DownCast( BaseClass::screen().scene()->object( "Bounds" ) );
        if ( bbox && Params::VisibleBoundingBox ) { bbox->setVisible( false ); }

        BaseClass::execRendering();

        const bool visible = BaseClass::world().isRoot();
        if ( bbox ) { bbox->setVisible( visible && Params::VisibleBoundingBox ); }

        if ( BaseClass::isEntropyStep() )
        {
            const auto index = BaseClass::maxIndex();
            const auto focus = BaseClass::maxFocusPoint();
            auto location = BaseClass::viewpoint().at( index );
            //add
            auto bestlocation = BaseClass::focusedLocation( location , focus );
            bestlocation.position = BaseClass::bestLocationPosition();
            bestlocation.rotation = BaseClass::maxRotation();
            //auto location = BaseClass::bestLocation();

            const auto level = BaseClass::bestZoomLevel();
            auto frame_buffer = BaseClass::readback( bestlocation );
            // Output the rendering images and the heatmap of entropies.
            if ( BaseClass::world().isRoot() )
            {
                if ( BaseClass::isOutputImageEnabled() )
                {
                    BaseClass::outputColorImage( bestlocation, frame_buffer, level );
                }
            }
        }
        else
        {
            const auto focus = BaseClass::erpFocus();
            auto location = BaseClass::erpLocation( focus );
            //location.position = BaseClass::bestLocationPosition();
            //auto bestlocation = BaseClass::erpLocation( focus );
            //auto location = BaseClass::bestLocation();
            const auto level = BaseClass::bestZoomLevel();
            auto frame_buffer = BaseClass::readback( location );

            // Output the rendering images and the heatmap of entropies.
            if ( BaseClass::world().isRoot() )
            {
                if ( BaseClass::isOutputImageEnabled() )
                {
                    BaseClass::outputColorImage( location, frame_buffer, level );
                }
            }
        }
    }
#endif

    void importBoundaryMesh( const std::string& filename )
    {
        m_boundary_mesh = kvs::PolygonImporter( filename );

        // Scaling coordinate values of the boundary object adjusing to
        // the coordinate scale of the volume dataset.
        const auto scale = 1.0f / 1000.0f;
        {
            auto coords = m_boundary_mesh.coords();
            for ( auto& p : coords ) { p *= scale; }
            m_boundary_mesh.setCoords( coords );
        }

        const auto min_coord = m_boundary_mesh.minObjectCoord() * scale;
        const auto max_coord = m_boundary_mesh.maxObjectCoord() * scale;
        m_boundary_mesh.setMinMaxObjectCoords( min_coord, max_coord );
        m_boundary_mesh.setMinMaxExternalCoords( min_coord, max_coord );

        // Removing x-max, y-min/max, z-min/max planes and setting normals.
        {
            auto v = m_boundary_mesh.coords();
            std::vector<kvs::Real32> coords;
            std::vector<kvs::Real32> normals;
            for ( size_t i = 0; i < v.size() / 9; i++ )
            {
                const auto v0 = kvs::Vec3( v[9*i+0], v[9*i+1], v[9*i+2] );
                const auto v1 = kvs::Vec3( v[9*i+3], v[9*i+4], v[9*i+5] );
                const auto v2 = kvs::Vec3( v[9*i+6], v[9*i+7], v[9*i+8] );
                const auto n0 = ( v1 - v0 ).cross( v2 - v0 ).normalized();

                // x-max
                if ( kvs::Math::Equal( max_coord.x(), v0.x() ) &&
                     kvs::Math::Equal( max_coord.x(), v1.x() ) &&
                     kvs::Math::Equal( max_coord.x(), v2.x() ) ) continue;
                // y-min
                if ( kvs::Math::Equal( min_coord.y(), v0.y() ) &&
                     kvs::Math::Equal( min_coord.y(), v1.y() ) &&
                     kvs::Math::Equal( min_coord.y(), v2.y() ) ) continue;

                // y-max
                if ( kvs::Math::Equal( max_coord.y(), v0.y() ) &&
                     kvs::Math::Equal( max_coord.y(), v1.y() ) &&
                     kvs::Math::Equal( max_coord.y(), v2.y() ) ) continue;

                // z-min
                if ( kvs::Math::Equal( min_coord.z(), v0.z() ) &&
                     kvs::Math::Equal( min_coord.z(), v1.z() ) &&
                     kvs::Math::Equal( min_coord.z(), v2.z() ) ) continue;

                // z-max
                if ( kvs::Math::Equal( max_coord.z(), v0.z() ) &&
                     kvs::Math::Equal( max_coord.z(), v1.z() ) &&
                     kvs::Math::Equal( max_coord.z(), v2.z() ) ) continue;

                coords.push_back( v0.x() );
                coords.push_back( v0.y() );
                coords.push_back( v0.z() );

                coords.push_back( v1.x() );
                coords.push_back( v1.y() );
                coords.push_back( v1.z() );

                coords.push_back( v2.x() );
                coords.push_back( v2.y() );
                coords.push_back( v2.z() );

                normals.push_back( n0.x() );
                normals.push_back( n0.y() );
                normals.push_back( n0.z() );
            }
            m_boundary_mesh.setCoords( kvs::ValueArray<kvs::Real32>( coords ) );
            m_boundary_mesh.setNormals( kvs::ValueArray<kvs::Real32>( normals ) );
        }
    }

    bool dump()
    {
        if ( !BaseClass::dump() ) return false;

#if defined( IN_SITU_VIS__CALCULATE_WHOLE_MIN_MAX_VALUES )
        log() << "Whole Min: " << m_whole_min_value << std::endl;
        log() << "Whole Max: " << m_whole_max_value << std::endl;
#endif

        // For each node
        m_sim_timer.setTitle( "Sim time" );
        m_cnv_timer.setTitle( "Cnv time" );
        m_vis_timer.setTitle( "Vis time" );

        const std::string rank = kvs::String::From( this->world().rank(), 4, '0' );
        const std::string subdir = BaseClass::outputDirectory().name() + "/";
        kvs::StampTimerList timer_list;
        timer_list.push( m_sim_timer );
        timer_list.push( m_cnv_timer );
        timer_list.push( m_vis_timer );
        if ( !timer_list.write( subdir + "proc_time_" + rank + ".csv" ) ) return false;

        // For root node
        auto sim_time_min = m_sim_timer; sim_time_min.reduceMin();
        auto sim_time_max = m_sim_timer; sim_time_max.reduceMax();
        auto sim_time_ave = m_sim_timer; sim_time_ave.reduceAve();
        auto cnv_time_min = m_cnv_timer; cnv_time_min.reduceMin();
        auto cnv_time_max = m_cnv_timer; cnv_time_max.reduceMax();
        auto cnv_time_ave = m_cnv_timer; cnv_time_ave.reduceAve();
        auto vis_time_min = m_vis_timer; vis_time_min.reduceMin();
        auto vis_time_max = m_vis_timer; vis_time_max.reduceMax();
        auto vis_time_ave = m_vis_timer; vis_time_ave.reduceAve();

        if ( !this->world().isRoot() ) return true;

        sim_time_min.setTitle( "Sim time (min)" );
        sim_time_max.setTitle( "Sim time (max)" );
        sim_time_ave.setTitle( "Sim time (ave)" );
        cnv_time_min.setTitle( "Cnv time (min)" );
        cnv_time_max.setTitle( "Cnv time (max)" );
        cnv_time_ave.setTitle( "Cnv time (ave)" );
        vis_time_min.setTitle( "Vis time (min)" );
        vis_time_max.setTitle( "Vis time (max)" );
        vis_time_ave.setTitle( "Vis time (ave)" );

        timer_list.clear();
        timer_list.push( sim_time_min );
        timer_list.push( sim_time_max );
        timer_list.push( sim_time_ave );
        timer_list.push( cnv_time_min );
        timer_list.push( cnv_time_max );
        timer_list.push( cnv_time_ave );
        timer_list.push( vis_time_min );
        timer_list.push( vis_time_max );
        timer_list.push( vis_time_ave );

        const auto basedir = BaseClass::outputDirectory().baseDirectoryName() + "/";
        return timer_list.write( basedir + "proc_time.csv" );
    }
};

inline InSituVis::Pipeline InSituVis::OrthoSlice()
{
    return [&] ( Screen& screen, const Object& object )
    {
        Volume volume; volume.shallowCopy( Volume::DownCast( object ) );
        if ( volume.numberOfCells() == 0 ) { return; }

        auto* mesh = kvs::PolygonObject::DownCast( screen.scene()->object( "BoundaryMesh" ) );
        if ( mesh )
        {
            const auto min_coord = mesh->minExternalCoord();
            const auto max_coord = mesh->maxExternalCoord();
            volume.setMinMaxObjectCoords( min_coord, max_coord );
            volume.setMinMaxExternalCoords( min_coord, max_coord );
        }

        // Setup a transfer function.
        const auto min_value = volume.minValue();
        const auto max_value = volume.maxValue();
        //auto t = kvs::TransferFunction( kvs::ColorMap::CoolWarm() );
        auto t = kvs::TransferFunction( kvs::ColorMap::BrewerSpectral() );
        t.setRange( min_value, max_value );

        // Create new slice objects.
        auto p0 = ( volume.minObjectCoord().y() + volume.maxObjectCoord().y() ) * 0.5f;
        auto a0 = kvs::OrthoSlice::YAxis;
        auto* object0 = new kvs::OrthoSlice( &volume, p0, a0, t );
        object0->setName( volume.name() + "Object0");

        auto p1 = ( volume.minObjectCoord().z() + volume.maxObjectCoord().z() ) * 0.5f;
        auto a1 = kvs::OrthoSlice::ZAxis;
        auto* object1 = new kvs::OrthoSlice( &volume, p1, a1, t );
        object1->setName( volume.name() + "Object1");

        if ( screen.scene()->hasObject( volume.name() + "Object0") )
        {
            // Update the objects.
            screen.scene()->replaceObject( volume.name() + "Object0", object0 );
            screen.scene()->replaceObject( volume.name() + "Object1", object1 );
        }
        else
        {
            // Register the objects with renderer.
            auto* renderer0 = new kvs::glsl::PolygonRenderer();
            auto* renderer1 = new kvs::glsl::PolygonRenderer();
            renderer0->setTwoSideLightingEnabled( true );
            renderer1->setTwoSideLightingEnabled( true );
            screen.registerObject( object0, renderer0 );
            screen.registerObject( object1, renderer1 );

            // Boundary mesh
            if ( Params::VisibleBoundaryMesh )
            {
                auto* renderer = new kvs::PolygonRenderer();
                renderer->setTwoSideLightingEnabled( true );
                screen.registerObject( mesh, renderer );
            }
        }
    };
}

inline InSituVis::Pipeline InSituVis::Isosurface()
{
    return [&] ( Screen& screen, const Object& object )
    {
        Volume volume; volume.shallowCopy( Volume::DownCast( object ) );
        if ( volume.numberOfCells() == 0 ) { return; }

        auto* mesh = kvs::PolygonObject::DownCast( screen.scene()->object( "BoundaryMesh" ) );
        if ( mesh )
        {
            const auto min_coord = mesh->minExternalCoord();
            const auto max_coord = mesh->maxExternalCoord();
            volume.setMinMaxObjectCoords( min_coord, max_coord );
            volume.setMinMaxExternalCoords( min_coord, max_coord );
        }

        // Setup a transfer function.
        const auto min_value = volume.minValue();
        const auto max_value = volume.maxValue();
        //auto t = kvs::TransferFunction( kvs::ColorMap::CoolWarm() );
        auto t = kvs::TransferFunction( kvs::ColorMap::BrewerSpectral() );
        t.setRange( min_value, max_value );

        // Create new object
        auto n = kvs::Isosurface::PolygonNormal;
        auto d = true;
        auto i0 = kvs::Math::Mix( min_value, max_value, 0.1 );
        auto* object0 = new kvs::Isosurface( &volume, i0, n, d, t );
        object0->setName( volume.name() + "Object0");

        auto i1 = kvs::Math::Mix( min_value, max_value, 0.3 );
        auto* object1 = new kvs::Isosurface( &volume, i1, n, d, t );
        object1->setName( volume.name() + "Object1");

        auto i2 = kvs::Math::Mix( min_value, max_value, 0.7 );
        auto* object2 = new kvs::Isosurface( &volume, i2, n, d, t );
        object2->setName( volume.name() + "Object2");

        // Register object and renderer to screen
        kvs::Light::SetModelTwoSide( true );
        if ( screen.scene()->hasObject( volume.name() + "Object0") )
        {
            // Update the objects.
            screen.scene()->replaceObject( volume.name() + "Object0", object0 );
            screen.scene()->replaceObject( volume.name() + "Object1", object1 );
            screen.scene()->replaceObject( volume.name() + "Object2", object2 );
        }
        else
        {
            // Register the objects with renderer.
            auto* renderer0 = new kvs::glsl::PolygonRenderer();
            auto* renderer1 = new kvs::glsl::PolygonRenderer();
            auto* renderer2 = new kvs::glsl::PolygonRenderer();
            renderer0->setTwoSideLightingEnabled( true );
            renderer1->setTwoSideLightingEnabled( true );
            renderer2->setTwoSideLightingEnabled( true );
            screen.registerObject( object0, renderer0 );
            screen.registerObject( object1, renderer1 );
            screen.registerObject( object2, renderer2 );

            // Boundary mesh
//            if ( Params::VisibleBoundaryMesh )
//            {
//                auto* renderer = new kvs::PolygonRenderer();
//                renderer->setTwoSideLightingEnabled( true );
//                screen.registerObject( mesh, renderer );
//            }
        }
    };
}

inline InSituVis::Pipeline InSituVis::ExternalFace( const kvs::mpi::Communicator& world )
{
    return [world] ( Screen& screen, const Object& object )
    {
        Volume volume; volume.shallowCopy( Volume::DownCast( object ) );
        if ( volume.numberOfCells() == 0 ) { return; }

        const auto* mesh = kvs::PolygonObject::DownCast( screen.scene()->object( "BoundaryMesh" ) );
        if ( mesh )
        {
            const auto min_coord = mesh->minExternalCoord();
            const auto max_coord = mesh->maxExternalCoord();
            volume.setMinMaxObjectCoords( min_coord, max_coord );
            volume.setMinMaxExternalCoords( min_coord, max_coord );
        }

        const auto cmap = kvs::ColorMap::BrewerSpectral( world.size() );
        auto indices = kvs::ValueArray<int>::Linear( world.size() );
        std::shuffle( indices.begin(), indices.end(), std::mt19937( 0 ) );
        const auto index = indices[ world.rank() ];
        const auto color = cmap[ index ];

        auto* faces = new kvs::ExternalFaces( &volume );
        faces->setName( volume.name() + "Object");
        faces->setColor( color );

        if ( screen.scene()->hasObject( volume.name() + "Object") )
        {
            // Update the objects.
            screen.scene()->replaceObject( volume.name() + "Object", faces );
        }
        else
        {
            // Register the objects with renderer.
            auto* renderer = new kvs::glsl::PolygonRenderer();
            renderer->setTwoSideLightingEnabled( true );
            screen.registerObject( faces, renderer );
        }
    };
}

inline InSituVis::Pipeline InSituVis::StochasticRendering( const size_t repeats )
{
    return [repeats] ( Screen& screen, const Object& object )
    {
        Volume volume; volume.shallowCopy( Volume::DownCast( object ) );
        if ( volume.numberOfCells() == 0 ) { return; }

        const auto* mesh = kvs::PolygonObject::DownCast( screen.scene()->object( "BoundaryMesh" ) );
        if ( mesh )
        {
            const auto min_coord = mesh->minExternalCoord();
            const auto max_coord = mesh->maxExternalCoord();
            volume.setMinMaxExternalCoords( min_coord, max_coord );
        }

        // Setup a transfer function.
        const auto min_value = volume.minValue();
        const auto max_value = volume.maxValue();

        //auto c = kvs::ColorMap::CoolWarm( 256 );
        auto c = kvs::ColorMap::BrewerSpectral( 256 );
        auto o = kvs::OpacityMap( 256 );
        o.addPoint(   0, 0.0 );
        o.addPoint(  30, 0.0 );
        o.addPoint(  50, 0.2 );
        o.addPoint( 100, 0.3 );
        o.addPoint( 240, 0.2 );
        o.addPoint( 245, 0.1 );
        o.addPoint( 255, 0.0 );
        o.create();
        auto t = kvs::TransferFunction( c, o );
        //auto t = kvs::TransferFunction( c );
        t.setRange( min_value, max_value );

        // Particle generation.
        using Sampler = kvs::CellByCellMetropolisSampling;
        const auto* camera = screen.scene()->camera();
        const auto step = 0.5f / 1000.0f;
        auto* point = new Sampler( camera, &volume, repeats, step, t );
        point->setName( volume.name() + "Object");

        if ( mesh )
        {
            const auto min_coord = mesh->minExternalCoord();
            const auto max_coord = mesh->maxExternalCoord();
            point->setMinMaxObjectCoords( min_coord, max_coord );
            point->setMinMaxExternalCoords( min_coord, max_coord );
        }

        // Register object and renderer to screen
        //kvs::Light::SetModelTwoSide( true );
        if ( screen.scene()->hasObject( volume.name() + "Object") )
        {
            // Update the objects.
            screen.scene()->replaceObject( volume.name() + "Object", point );
        }
        else
        {
            // Register the objects with renderer.
            auto* point_renderer = new kvs::glsl::ParticleBasedRenderer();
            point_renderer->setTwoSideLightingEnabled( true );
            screen.registerObject( point, point_renderer );
        }
    };
}

} // end of namspace local