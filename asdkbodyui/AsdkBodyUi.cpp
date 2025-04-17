//////////////////////////////////////////////////////////////////////////////
//
//  Copyright 2023 Autodesk, Inc.  All rights reserved.
//
//  Use of this software is subject to the terms of the Autodesk license 
//  agreement provided at the time of installation or download, or which 
//  otherwise accompanies this software in either electronic or hard copy form.   
//
//////////////////////////////////////////////////////////////////////////////


#if defined(_DEBUG) && !defined(_FULLDEBUG_)
#define _DEBUG_WAS_DEFINED
#pragma message ("       Compiling MFC /STL /ATL header files in release mode.")
#undef _DEBUG
#endif

#include <Windows.h>
#include "dbmain.h"
#include "accmd.h"
#include "rxregsvc.h"                   // For acrxRegisterService 
#include "adslib.h"
#include "actrans.h"
#include "assert.h"
#include "dbsymtb.h"
#include "dbents.h"
#include "dbpl.h"
#include "geassign.h"


#include "AsdkBodyUi.h"
#include "..\AsdkBodyObj\Asdkbody.h"
#include "errormsg.h"
#include "dbxutil.h"
#include "string.h"
#include "tchar.h"
#include "acedCmdNF.h"
#include <unordered_map>

#ifdef _DEBUG_WAS_DEFINED
#define _DEBUG
#undef _DEBUG_WAS_DEFINED
#endif

// 
// Function prototypes
// 
void printHelp();
void createBox();
void createSphere();
void createCylinder();
void createCone();
void createPipe();
void createPipeConic();
void createTetrahedron();
void createTorus();
void createReducingElbow();
void createRectToCircleReducer();
void createConvexHull();
void createFace();
void createPyramid();
void createExtrusion();
void createAxisRevolution();
void createExtrusionAlongPath();
void aUnion();
void aSubtract();
void aIntersect();
void doArray();
void toObj();
void doSubstract();
void test202513();

//
// Static function prototypes
//
static AcDbEntity* selectEntity(AcDb::OpenMode openMode);
static bool getYesNo(bool defaultIsYes = TRUE);
static void cleanupEntity(AcDbEntity *pEnt);
static void handleADSError(int adsrc);
static AsdkBody* getFace();
static AsdkBody* getExistingFace();
static AsdkBody* getNewFace(ACHAR* kw);
static void getVertices(AcGePoint3d* &vertices, int &iVertices, int iMaxVertices = 64);
static void getFilletVertexData(const int iVertices, PolygonVertexData** &vertexData);
static AcDb2dPolyline* getPolyline();
static AcGeVector3d getVector();
static void getTwoPoints(AcGePoint3d& p1, AcGePoint3d &p2);
static AcGePoint3d getPoint(ACHAR* prompt = ACRX_T(""));
static double getDistance(AcGePoint3d fromPt, ACHAR *prompt = ACRX_T(""), double defaultDist = 0.0);
static int getInt(ACHAR *prompt = ACRX_T(""), int defaultValue = 0);
static double getReal(ACHAR *prompt = ACRX_T(""), double defaultValue = 0.0);
static void getPolylineVertices(AcGePoint3d* &vertices, 
    PolygonVertexData** &vertexData, int &iVertices, AcGeVector3d &normal);
static void getPath(AcGePoint3d* &vertices, PolygonVertexData** &vertexData, 
    int &iVertices);
static MorphingMap getMorphingMap(const Body& startProfile, const Body& endProfile);
static MorphingMap* queryMorphingMap(AsdkBody* pBody1, AsdkBody* pBody2);



// -------------------------------------------------
// -------------------------------------------------
//    Initialization
// -------------------------------------------------
// -------------------------------------------------
//  This data structure holds info about commands to be
//  registered and to be listed by the ifcHelp command
//
CommandInfo commandInfo[] =
{
    // 
    // commands
    // 
    ACRX_T("AHELP"),                &printHelp,                 ACRX_T("Display this command table"),
    ACRX_T("ABOX"),                 &createBox,                 ACRX_T("Create a Box"), 
    ACRX_T("ASPHERE"),              &createSphere,              ACRX_T("Create a Sphere"), 
    ACRX_T("ACYLINDER"),            &createCylinder,            ACRX_T("Create a Cylinder"),
    ACRX_T("ACONE"),                &createCone,                ACRX_T("Create a Cone"),
    ACRX_T("APIPE"),		        &createPipe,                ACRX_T("Create a Pipe"),
	ACRX_T("APIPECONIC"),           &createPipeConic,           ACRX_T("Create a Pipe Conic"),
	ACRX_T("ATETRAHEDRON"),         &createTetrahedron,         ACRX_T("Create a Tetrahedron"),
	ACRX_T("ATORUS"),               &createTorus,               ACRX_T("Create a Torus"),
	ACRX_T("AREDUCINGELBOW"),       &createReducingElbow,       ACRX_T("Create a Reducing Elbow"),
	ACRX_T("ARECTTOCIRCLEREDUCER"), &createRectToCircleReducer, ACRX_T("Create a Circle Reducer"),
	ACRX_T("ACONVEXHULL"),          &createConvexHull,          ACRX_T("Create a Convex Hull"),
    ACRX_T("AFACE"),		        &createFace,	            ACRX_T("Create a Face"),
	ACRX_T("APYRAMID"),             &createPyramid,             ACRX_T("Create a Pyramid"),
	ACRX_T("AEXTRUDE"),             &createExtrusion,           ACRX_T("Sweep along a straight line"),
    ACRX_T("AAXISREVOLVE"),         &createAxisRevolution,      ACRX_T("Sweep around an axis"),
    ACRX_T("APATHEXTRUDE"),         &createExtrusionAlongPath,  ACRX_T("Sweep along a path"),
    ACRX_T("AUNION"),               &aUnion,                    ACRX_T("Unite Two AsdkBodys"), 
    ACRX_T("ASUBTRACT"),            &aSubtract,                 ACRX_T("Subtract Two AsdkBodys"),
    ACRX_T("AINTERSECT"),           &aIntersect,                ACRX_T("Intersects AsdkBodys"),
    ACRX_T("DOARRAY"),              &doArray,                   ACRX_T("Do an Array operation"),
    ACRX_T("toObj"),                &toObj,                     ACRX_T("输出"),
    ACRX_T("doSubstract"),       & doSubstract,                     ACRX_T("输出1"),
    ACRX_T("test202513"),& test202513,                     ACRX_T("test2025131"),
    0,                      0,                          0
};

//utility function that appends an entity to the model space
Adesk::Boolean append( AcDbEntity* pEnt )
{
    AcDbBlockTable *pBt;
	AcDbDatabase* pDb = acdbHostApplicationServices()->workingDatabase();
    if (Acad::eOk != pDb->getBlockTable( pBt, AcDb::kForRead ))
        return Adesk::kFalse;

    AcDbBlockTableRecord *pMs;
    Acad::ErrorStatus es = pBt->getAt( ACDB_MODEL_SPACE, pMs,
        AcDb::kForWrite );
    pBt->close();
    if (Acad::eOk != es)
        return Adesk::kFalse;
    es = pMs->appendAcDbEntity( pEnt );
    pMs->close();
    return Acad::eOk == es;
}


const ACHAR* gstrCmdGrpName = ACRX_T("ASDKBODYUI_COMMANDS");

extern "C" AcRx::AppRetCode
acrxEntryPoint( AcRx::AppMsgCode msg, void *pkt )
{

    switch(msg)
    {
    case AcRx::kInitAppMsg:
		{
		//Initialize the server app
        acrxLoadModule(ACRX_T("AsdkBodyObj.dbx"),1);
		for (CommandInfo * pInfo = commandInfo;  pInfo->commandName != NULL; pInfo++)
		{
			if (acedRegCmds->addCommand(gstrCmdGrpName, pInfo->commandName,
				pInfo->commandName, ACRX_CMD_MODAL, pInfo->fn ) 
				!= Acad::eOk )
			{
				ads_printf ( ACRX_T("There was a problem with addCommand().") );
				break;
			}
		}
        // 
        // Try to allow unloading
        // 
        acrxUnlockApplication(pkt);
		//register as being MDI aware
		acrxRegisterAppMDIAware(pkt);
        printHelp();
        break;
		}
    case AcRx::kUnloadAppMsg:
        // 
		// Clean up the commands that were registered with the editor.
		//
		acedRegCmds->removeGroup ( gstrCmdGrpName );
		acrxUnloadModule(ACRX_T("AsdkBodyObj.dbx"));
        break;
    }

    return AcRx::kRetOK;
}


//
//  ArxAppHelp()
//
//  Print out the short descriptions of all application commands.
//
void 
printHelp()
{
    ads_printf(ACRX_T("\n\n%-20s%s\n"), ACRX_T("Commands"), ACRX_T("Descriptions"));
    ads_printf(ACRX_T("--------------------------------------------------------\n"));

    for (CommandInfo * pInfo = commandInfo; pInfo->commandName != NULL;  pInfo++)
        ads_printf ( ACRX_T("%-20s%s\n"), pInfo->commandName, pInfo->desc );
}



////////////////////////////////////////////////////////////////////////////////
//
// AsdkBody primitve creation functions
//


void 
createBox()
{
    AsdkBody *pBody = NULL;

    try
    {
        AcGePoint3d corner1 = getPoint(ACRX_T("\nCorner of box: "));
        AcGePoint3d corner2 = getPoint(ACRX_T("\nOther corner: "));
        ads_printf( ACRX_T("\n") );
    
        pBody = new AsdkBody;

        AcGeVector3d vec = corner2 - corner1;
        if (vec[X] == 0.0)
            vec[X] = getDistance(corner2, ACRX_T("\nWidth: "));
        if (vec[Y] == 0.0)
            vec[Y] = getDistance(corner2, ACRX_T("\nDepth: "));
        if (vec[Z] == 0.0)
            vec[Z] = getDistance(corner2, ACRX_T("\nHeight: "));

        pBody->createBox(corner1, vec);
    }
    catch (int caught_adsrc)
    //
    // User probably cancelled.  Clean up and bail out.
    {
        handleADSError(caught_adsrc);
        return;
    }
    catch (ErrorCode err) 
    {
        if (err == eFail)
        {
            ads_printf( ACRX_T("\nBody is not valid.  Continue?"));
            if(! getYesNo())
            {
                delete pBody;
                return;
            }
        }
        else
        {
            ads_printf( ACRX_T("\nERROR: %s\n"), getErrorMsg(err));
            delete pBody;
            return;
        }
    }
    if (append( pBody ))
        pBody->close();
    else {
        delete pBody;
        ads_printf( ACRX_T("\nCould not add AsdkBody to ACAD Database.  *Bailing out*\n") );
    }
}


void 
createSphere()
{
    AsdkBody *pBody = NULL;

    try
    {
        AcGePoint3d center = getPoint(ACRX_T("\nCenter of sphere: "));
        double dist = getDistance(center, ACRX_T("\nRadius of sphere: "));
        int approx = getInt(ACRX_T("\nEnter # of lines to approximate a circle <32>:"), 32);
    
        pBody = new AsdkBody;
    
        pBody->createSphere(center, dist, approx);
    }
    catch (int caught_adsrc)
    //
    // User probably cancelled.  Clean up and bail out.
    {
        handleADSError(caught_adsrc);
        return;
    }
    catch (ErrorCode err) 
    {
        if (err == eFail)
        {
            ads_printf( ACRX_T("\nBody is not valid.  Continue?"));
            if(! getYesNo())
            {
                delete pBody;
                return;
            }
        }
        else
        {
            ads_printf( ACRX_T("\nERROR: %s\n"), getErrorMsg(err));
            delete pBody;
            return;
        }
    }

    if (append( pBody ))
        pBody->close();
    else {
        delete pBody;
        ads_printf( ACRX_T("\nCould not add AsdkBody to ACAD Database.  *Bailing out*\n") );
    }
}


void 
createCylinder()
{
    AsdkBody *pBody = NULL;

    try
    {
        AcGePoint3d axisStart, axisEnd;
        ads_printf(ACRX_T("\nCylinder axis: "));
        getTwoPoints(axisStart, axisEnd);

        AcGeVector3d baseNormal;
        try
        {
            baseNormal = getPoint(ACRX_T("\nBase normal vector <None>:")) - axisStart;
        }
        catch (int caught_adsrc)
        {
            if (caught_adsrc == RTNONE)
                baseNormal = AcGeVector3d(0, 0, 0);
            else
                throw caught_adsrc;
        }
        double radius = getDistance(axisStart, ACRX_T("\nRadius: "));
        int approx = getInt(ACRX_T("\nEnter # of lines to approximate a circle <32>:"), 32);

        pBody = new AsdkBody;
    
        pBody->createCylinder(axisStart, axisEnd, baseNormal, radius, approx);
    }
    catch (int caught_adsrc)
    //
    // User probably cancelled.  Clean up and bail out.
    {
        handleADSError(caught_adsrc);
        return;
    }
    catch (ErrorCode err) 
    {
        if (err == eFail)
        {
            ads_printf( ACRX_T("\nBody is not valid.  Continue?"));
            if(! getYesNo())
            {
                delete pBody;
                return;
            }
        }
        else
        {
            ads_printf( ACRX_T("\nERROR: %s\n"), getErrorMsg(err));
            delete pBody;
            return;
        }
    }

    if (append( pBody ))
        pBody->close();
    else {
        delete pBody;
        ads_printf( ACRX_T("\nCould not add AsdkBody to ACAD Database.  *Bailing out*\n") );
    }
}


void 
createCone()
{
    AsdkBody *pBody = NULL;

    try
    {
        AcGePoint3d axisStart, axisEnd;
        ads_printf(ACRX_T("\nCone axis: "));
        getTwoPoints(axisStart, axisEnd);

        AcGeVector3d baseNormal;
        try
        {
            baseNormal = getPoint(ACRX_T("\nBase normal vector <None>: ")) - axisStart;
        }
        catch (int caught_adsrc)
        {
            if (caught_adsrc == RTNONE)
                baseNormal = AcGeVector3d(0, 0, 0);
            else
                throw caught_adsrc;
        }
        double radius1 = getDistance(axisStart, ACRX_T("\nStart radius: "));
        double radius2 = getDistance(axisEnd, ACRX_T("\nEnd radius: "));
        int approx = getInt(ACRX_T("\nEnter # of lines to approximate a circle <32>:"), 32);

        pBody = new AsdkBody;
    
        pBody->createCone(axisStart, axisEnd, baseNormal, 
            radius1, radius2, approx);
    }
    catch (int caught_adsrc)
    //
    // User probably cancelled.  Clean up and bail out.
    {
        handleADSError(caught_adsrc);
        return;
    }
    catch (ErrorCode err) 
    {
        if (err == eFail)
        {
            ads_printf( ACRX_T("\nBody is not valid.  Continue?"));
            if(! getYesNo())
            {
                delete pBody;
                return;
            }
        }
        else
        {
            ads_printf( ACRX_T("\nERROR: %s\n"), getErrorMsg(err));
            delete pBody;
            return;
        }
    }

    if (append( pBody ))
        pBody->close();
    else {
        delete pBody;
        ads_printf( ACRX_T("\nCould not add AsdkBody to ACAD Database.  *Bailing out*\n") );
    }
}


void 
createPipe()
{
    AsdkBody *pBody = NULL;

    try
    {
        AcGePoint3d axisStart, axisEnd;
        ads_printf(ACRX_T("\nPipe axis: "));
        getTwoPoints(axisStart, axisEnd);

        AcGeVector3d baseNormal;
        try
        {
            baseNormal = getPoint(ACRX_T("\nBase normal vector <None>:")) - axisStart;
        }
        catch (int caught_adsrc)
        {
            if (caught_adsrc == RTNONE)
                baseNormal = AcGeVector3d(0, 0, 0);
            else
                throw caught_adsrc;
        }
        double outerRadius = getDistance(axisStart, ACRX_T("\nOuter radius: "));
        double innerRadius = getDistance(axisStart, ACRX_T("\nInner radius: "));
        int approx = getInt(ACRX_T("\nEnter # of lines to approximate a circle <32>:"), 32);
    
        pBody = new AsdkBody;
    
        pBody->createPipe(axisStart, axisEnd, baseNormal, 
            outerRadius, innerRadius, approx);
    }
    catch (int caught_adsrc)
    //
    // User probably cancelled.  Clean up and bail out.
    {
        handleADSError(caught_adsrc);
        return;
    }
    catch (ErrorCode err) 
    {
        if (err == eFail)
        {
            ads_printf( ACRX_T("\nBody is not valid.  Continue?"));
            if(! getYesNo())
            {
                delete pBody;
                return;
            }
        }
        else
        {
            ads_printf( ACRX_T("\nERROR: %s\n"), getErrorMsg(err));
            delete pBody;
            return;
        }
    }

    if (append( pBody ))
        pBody->close();
    else {
        delete pBody;
        ads_printf( ACRX_T("\nCould not add AsdkBody to ACAD Database.  *Bailing out*\n") );
    }
}


void 
createPipeConic()
{
    AsdkBody *pBody = NULL;

    try
    {
        AcGePoint3d axisStart, axisEnd;
        ads_printf(ACRX_T("\nConic pipe axis: "));
        getTwoPoints(axisStart, axisEnd);

        AcGeVector3d baseNormal;
        try
        {
            baseNormal = getPoint(ACRX_T("\nBase normal vector <None>:")) - axisStart;
        }
        catch (int caught_adsrc)
        {
            if (caught_adsrc == RTNONE)
                baseNormal = AcGeVector3d(0, 0, 0);
            else
                throw caught_adsrc;
        }
        double outerRadius1 = getDistance(axisStart, ACRX_T("\nFirst outer radius: "));
        double innerRadius1 = getDistance(axisStart, ACRX_T("\nFirst inner radius: "));
        double outerRadius2 = getDistance(axisEnd, ACRX_T("\nSecond outer radius: "));
        double innerRadius2 = getDistance(axisEnd, ACRX_T("\nSecond inner radius: "));
        int approx = getInt(ACRX_T("\nEnter # of lines to approximate a circle <32>:"), 32);
    
        pBody = new AsdkBody;
    
         pBody->createPipeConic(axisStart, axisEnd, baseNormal,
            outerRadius1, innerRadius1, outerRadius2, innerRadius2, 
            approx);
    }
    catch (int caught_adsrc)
    //
    // User probably cancelled.  Clean up and bail out.
    {
        handleADSError(caught_adsrc);
        return;
    }
    catch (ErrorCode err) 
    {
        if (err == eFail)
        {
            ads_printf( ACRX_T("\nBody is not valid.  Continue?"));
            if(! getYesNo())
            {
                delete pBody;
                return;
            }
        }
        else
        {
            ads_printf( ACRX_T("\nERROR: %s\n"), getErrorMsg(err));
            delete pBody;
            return;
        }
    }

    if (append( pBody ))
        pBody->close();
    else {
        delete pBody;
        ads_printf( ACRX_T("\nCould not add AsdkBody to ACAD Database.  *Bailing out*\n") );
    }
}


void 
createTetrahedron()
{
    AsdkBody *pBody = NULL;

    try
    {
        AcGePoint3d corner1 = getPoint(ACRX_T("\nCorner one: "));
        AcGePoint3d corner2 = getPoint(ACRX_T("\nCorner two: "));
        AcGePoint3d corner3 = getPoint(ACRX_T("\nCorner three: "));
        AcGePoint3d corner4 = getPoint(ACRX_T("\nCorner four: "));
        ads_printf( ACRX_T("\n") );
    
        pBody = new AsdkBody;

        pBody->createTetrahedron(corner1, corner2, corner3, corner4);
    }
    catch (int caught_adsrc)
    //
    // User probably cancelled.  Clean up and bail out.
    {
        handleADSError(caught_adsrc);
        return;
    }
    catch (ErrorCode err) 
    {
        if (err == eFail)
        {
            ads_printf( ACRX_T("\nBody is not valid.  Continue?"));
            if(! getYesNo())
            {
                delete pBody;
                return;
            }
        }
        else
        {
            ads_printf( ACRX_T("\nERROR: %s\n"), getErrorMsg(err));
            delete pBody;
            return;
        }
    }

    if (append( pBody ))
        pBody->close();
    else {
        delete pBody;
        ads_printf( ACRX_T("\nCould not add AsdkBody to ACAD Database.  *Bailing out*\n") );
    }
}


void 
createTorus()
{
    AsdkBody *pBody = NULL;

    try
    {
        AcGePoint3d axisStart, axisEnd;
        ads_printf(ACRX_T("\nTorus axis: "));
        getTwoPoints(axisStart, axisEnd);

        double majorRadius = getDistance(axisStart, ACRX_T("\nMajor radius: "));
        double minorRadius = getDistance(axisStart, ACRX_T("\nMinor radius: "));
        int majorApprox = getInt(ACRX_T("\nEnter # of lines to approximate major radius <32>:"), 32);
        int minorApprox = getInt(ACRX_T("\nEnter # of lines to approximate minor radius <32>:"), 32);
    
        pBody = new AsdkBody;
    
        pBody->createTorus(axisStart, axisEnd, majorRadius, minorRadius, 
            majorApprox, minorApprox );
    }
    catch (int caught_adsrc)
    //
    // User probably cancelled.  Clean up and bail out.
    {
        handleADSError(caught_adsrc);
        return;
    }
    catch (ErrorCode err) 
    {
        if (err == eFail)
        {
            ads_printf( ACRX_T("\nBody is not valid.  Continue?"));
            if(! getYesNo())
            {
                delete pBody;
                return;
            }
        }
        else
        {
            ads_printf( ACRX_T("\nERROR: %s\n"), getErrorMsg(err));
            delete pBody;
            return;
        }
    }

    if (append( pBody ))
        pBody->close();
    else {
        delete pBody;
        ads_printf( ACRX_T("\nCould not add AsdkBody to ACAD Database.  *Bailing out*\n") );
    }
}


void 
createReducingElbow()
{
    AsdkBody *pBody = NULL;

    try
    {
        AcGePoint3d elbowCenter = getPoint(ACRX_T("\nElbow center: "));
        AcGePoint3d endCenter1 = getPoint(ACRX_T("\nEnd one center: "));
        double endRadius1 = getDistance(endCenter1, ACRX_T("\nEnd one radius: "));
        AcGePoint3d endCenter2 = getPoint(ACRX_T("\nEnd two center: "));
        double endRadius2 = getDistance(endCenter2, ACRX_T("\nEnd two radius: "));
        int majorApprox = getInt(ACRX_T("\nEnter # of lines to approximate major radius <32>:"), 32);
        int minorApprox = getInt(ACRX_T("\nEnter # of lines to approximate minor radius <32>:"), 32);
    
        pBody = new AsdkBody;
    
        pBody->createReducingElbow(elbowCenter, endCenter1, endCenter2, 
            endRadius1, endRadius2, majorApprox, minorApprox);
    }
    catch (int caught_adsrc)
    //
    // User probably cancelled.  Clean up and bail out.
    {
        handleADSError(caught_adsrc);
        return;
    }
    catch (ErrorCode err) 
    {
        if (err == eFail)
        {
            ads_printf( ACRX_T("\nBody is not valid.  Continue?"));
            if(! getYesNo())
            {
                delete pBody;
                return;
            }
        }
        else
        {
            ads_printf( ACRX_T("\nERROR: %s\n"), getErrorMsg(err));
            delete pBody;
            return;
        }
    }

    if (append( pBody ))
        pBody->close();
    else {
        delete pBody;
        ads_printf( ACRX_T("\nCould not add AsdkBody to ACAD Database.  *Bailing out*\n") );
    }
}


void 
createRectToCircleReducer()
{
    AsdkBody *pBody = NULL;

    try
    {
        AcGePoint3d baseCorner = getPoint(ACRX_T("\nBase corner: "));
        AcGeVector3d baseSizes3d = getPoint(ACRX_T("\nBase sizes: ")) - baseCorner;
        AcGeVector2d baseSizes(baseSizes3d.x, baseSizes3d.y);
        AcGePoint3d circleCenter = getPoint(ACRX_T("\nCircle center: "));
        double circleRadius = getDistance(circleCenter, ACRX_T("\nCircle radius: "));
        AcGeVector3d circleNormal;

        // Get circle normal.  Jiri changed it so that it always uses the
        // Z axis, so this really does nothing.  -vm
        //
        try
        {
            circleNormal 
                = getPoint(ACRX_T("\nCircle normal vector <Z-Axis>: ")) - AcGePoint3d(0, 0, 0);
        }
        catch (int caught_adsrc)
        {
            if (caught_adsrc == RTNONE)
                circleNormal = AcGeVector3d::kZAxis;
            else
                throw caught_adsrc;
        }

        int approx = getInt(ACRX_T("\nEnter # of lines to approximate a circle <32>:"), 32);
    
        pBody = new AsdkBody;
    
        pBody->createRectToCircleReducer( baseCorner, baseSizes,
	        circleCenter, circleNormal, circleRadius, approx);
    }
    catch (int caught_adsrc)
    //
    // User probably cancelled.  Clean up and bail out.
    {
        handleADSError(caught_adsrc);
        return;
    }
    catch (ErrorCode err) 
    {
        if (err == eFail)
        {
            ads_printf( ACRX_T("\nBody is not valid.  Continue?"));
            if(! getYesNo())
            {
                delete pBody;
                return;
            }
        }
        else
        {
            ads_printf( ACRX_T("\nERROR: %s\n"), getErrorMsg(err));
            delete pBody;
            return;
        }
    }

    if (append( pBody ))
        pBody->close();
    else {
        delete pBody;
        ads_printf( ACRX_T("\nCould not add AsdkBody to ACAD Database.  *Bailing out*\n") );
    }
}

void 
createConvexHull()
{
    AsdkBody *pBody = NULL;
    AcGePoint3d *vertices = NULL;
    int iVertices;

    try
    {
        getVertices(vertices, iVertices);

		pBody = new AsdkBody;

//		pBody->createFace(vertices, iVertices);
		pBody->createConvexHull(vertices, iVertices);
    }
    catch(int caught_adsrc)
    //
    // User probably cancelled.  Clean up and bail out.
    {
        handleADSError(caught_adsrc);
        goto cleanup;
    }
	catch (ErrorCode err) 
    {
        if (err == eFail)
        {
            ads_printf( ACRX_T("\nBody is not valid.  Continue?"));
            if(! getYesNo())
            {
                delete pBody;
                goto cleanup;
            }
        }
        else
        {
            ads_printf( ACRX_T("\nERROR: %s\n"), getErrorMsg(err));
            delete pBody;
            goto cleanup;
        }
    }

	if (append( pBody ))
        pBody->close();
    else {
        delete pBody;
        ads_printf( ACRX_T("\nCould not add AsdkBody to ACAD Database.  *Bailing out*\n") );
    }

cleanup:

    delete [] vertices;
}


//
// createFace
//
// Gets a single-face AsdkBody object, then closes it.
//
// (If the user chooses to get an existing face, the
// net effect of this function is nada -- I think...)
//
void 
createFace()
{
	AsdkBody* pBody = NULL;
    
    try
    {
        pBody = getFace();
    }
    catch (int caught_adsrc)
    {
        handleADSError(caught_adsrc);
        return;
    }
    catch (ErrorCode err) 
    {
        ads_printf( ACRX_T("\nERROR: %s\n"), getErrorMsg(err));
        delete pBody;
        return;
    }

	if (pBody != NULL)
    {
		pBody->close();
    }
}


void 
createPyramid()
{
    AsdkBody *pBody = NULL;
    AcGePoint3d *vertices = NULL;
	PolygonVertexData** vertexData = NULL;
	int iVertices;
    AcGeVector3d normal;
    AcGePoint3d apex(0,0,1);
    
    try 
    {
        ads_printf(ACRX_T("\nProfile to extrude: "));
        while (vertices == NULL)
            getPolylineVertices(vertices, vertexData, iVertices, normal);
        
        apex = getPoint(ACRX_T("\nApex point: "));
	    
        pBody = new AsdkBody;
    
		pBody->createPyramid(vertices, vertexData, iVertices, normal, apex);
	}
    catch (int caught_adsrc)
    //
    // User probably cancelled.  Clean up and bail out.
    {
        handleADSError(caught_adsrc);
        goto cleanup;
    }
    catch (ErrorCode err) 
    {
        if (err == eFail)
        {
            ads_printf( ACRX_T("\nBody is not valid.  Continue?"));
            if(! getYesNo())
            {
                delete pBody;
                goto cleanup;
            }
        }
        else
        {
            ads_printf( ACRX_T("\nERROR: %s\n"), getErrorMsg(err));
            delete pBody;
            goto cleanup;
        }
    }
    if (append( pBody ))
        pBody->close();
    else {
        delete pBody;
        ads_printf( ACRX_T("\nCould not add AsdkBody to ACAD Database.  *Bailing out*\n") );
    }

cleanup:
    
    delete [] vertices;
    delete [] vertexData;
}


void 
createExtrusion()
{
    AsdkBody *pBody = NULL;
    AcGePoint3d *vertices = NULL;
	PolygonVertexData** vertexData = NULL;
	int iVertices;
    AcGeVector3d normal;
    AcGeVector3d extrusion;
    double scaleFactor, twistAngle;
    Point3d fixedPt = Point3d::kNull;
    
    try 
    {
        ads_printf(ACRX_T("\nProfile to extrude: "));
        while (vertices == NULL)
            getPolylineVertices(vertices, vertexData, iVertices, normal);

        ads_printf(ACRX_T("\nExtrusion vector: "));
        extrusion = getVector();

        // Get scale factor and twist angle
        //
        scaleFactor = getReal(ACRX_T("\nScale factor <1.0>: "), 1.0);
        twistAngle = getReal(ACRX_T("\nTwist angle <0.0>: "), 0.0);
        if (scaleFactor != 1.0 || twistAngle != 0.0)
        {
            try {
                fixedPt = getPoint(ACRX_T("Select fixed point for scale/twist <0,0,0>: "));
            }
            catch (int caught_adsrc)
            {
                if (caught_adsrc)
                    fixedPt = Point3d::kNull;
                else
                    throw caught_adsrc;
            }
        }
		else
			fixedPt = vertices[0];
        pBody = new AsdkBody;
    
		pBody->createExtrusion(vertices, vertexData, iVertices, normal, 
            extrusion, fixedPt, scaleFactor, twistAngle);
	}
    catch (int caught_adsrc)
    //
    // User probably cancelled.  Clean up and bail out.
    {
        handleADSError(caught_adsrc);
        goto cleanup;
    }
    catch (ErrorCode err) 
    {
        if (err == eFail)
        {
            ads_printf( ACRX_T("\nBody is not valid.  Continue?"));
            if(! getYesNo())
            {
                delete pBody;
                goto cleanup;
            }
        }
        else
        {
            ads_printf( ACRX_T("\nERROR: %s\n"), getErrorMsg(err));
            delete pBody;
            goto cleanup;
        }
    }
    if (append( pBody ))
        pBody->close();
    else {
        delete pBody;
        ads_printf( ACRX_T("\nCould not add AsdkBody to ACAD Database.  *Bailing out*\n") );
    }

cleanup:
    
    delete [] vertices;
    delete [] vertexData;
}


void 
createAxisRevolution()
{
    AsdkBody *pBody = NULL;
    AcGePoint3d *vertices = NULL;
	PolygonVertexData** vertexData = NULL;
	int iVertices;
    AcGeVector3d normal;
    AcGePoint3d axisStart(0,0,0);
    AcGePoint3d axisEnd(1,1,0);
    ads_real revolutionAngle;
    int approx;
    Point3d fixedPt = Point3d::kNull;
    double scaleFactor;
    double twistAngle;
    
    try 
    {
        ads_printf(ACRX_T("\nProfile to revolve: "));
        while (vertices == NULL)
            getPolylineVertices(vertices, vertexData, iVertices, normal);

        ads_printf(ACRX_T("\nRotation axis: "));
        getTwoPoints(axisStart, axisEnd);

        revolutionAngle = getReal(ACRX_T("\nRevolution angle <360>: "), 360.0);

        approx = getInt(ACRX_T("\nEnter # of lines to approximate a circle <32>:"), 32);

        // Get scale factor and twist angle
        //
        scaleFactor = getReal(ACRX_T("\nScale factor <1.0>: "), 1.0);
        twistAngle = getReal(ACRX_T("\nTwist angle <0.0>: "), 0.0);
        if (scaleFactor != 1.0 || twistAngle != 0.0)
        {
            try {
                fixedPt = getPoint(ACRX_T("Select fixed point for scale/twist <0,0,0>: "));
            }
            catch (int caught_adsrc)
            {
                if (caught_adsrc)
                    fixedPt = Point3d::kNull;
                else
                    throw caught_adsrc;
            }
        }

        pBody = new AsdkBody;
    
		pBody->createAxisRevolution(vertices, vertexData, iVertices, 
            normal, axisStart, axisEnd, (double)revolutionAngle, approx,
            fixedPt, (double)scaleFactor, (double)twistAngle);
	}
    catch (int caught_adsrc)
    //
    // User probably cancelled.  Clean up and bail out.
    {
        handleADSError(caught_adsrc);
        goto cleanup;
    }
    catch (ErrorCode err) 
    {
        if (err == eFail)
        {
            ads_printf( ACRX_T("\nBody is not valid.  Continue?"));
            if(! getYesNo())
            {
                delete pBody;
                goto cleanup;
            }
        }
        else
        {
            ads_printf( ACRX_T("\nERROR: %s\n"), getErrorMsg(err));
            delete pBody;
            goto cleanup;
        }
    }
    if (append( pBody ))
        pBody->close();
    else {
        delete pBody;
        ads_printf( ACRX_T("\nCould not add AsdkBody to ACAD Database.  *Bailing out*\n") );
    }

cleanup:
    
    delete [] vertices;
    delete [] vertexData;
}


//
// createExtrusionPath()
//
// Creates a CBody object from one or two profile faces
// (single-face AsdkBody objects) and an extrusion path.
//
void 
createExtrusionAlongPath()
{
    AsdkBody *pBody = NULL;
    AsdkBody *pStartFace = NULL, *pEndFace = NULL;
    MorphingMap *pMorphMap = NULL;
    AcGePoint3d *vertices = NULL;
    PolygonVertexData** vertexData = NULL;
    int iVertices;
    bool bClosePath;
    bool bCheckValidity;
    double scaleFactor, twistAngle;
    Point3d fixedPt = Point3d::kNull;

    try
    {
        ads_printf( ACRX_T("\nStart profile: ") );
        pStartFace = getFace();

        ads_printf( ACRX_T("\nEnd profile: ") );
        pEndFace = getFace();

        if( pEndFace == NULL ) {
            pEndFace = new AsdkBody;  // empty body
            ads_printf(ACRX_T("\nNull end profile will be used."));
        }

        // Get morphing map if there is an end face
        //
        // The const_cast is necessary. We need to make it a const,
        // so it calls the right verson of body() which calls assertReadEnabled().
        // pEndFace is currently open for read.
        //
        if (!const_cast<const AsdkBody*>(pEndFace)->body().isNull())
            pMorphMap = queryMorphingMap(pStartFace, pEndFace);
        if (pMorphMap == NULL)
            pMorphMap = new MorphingMap(MorphingMap::kNull);

        // Get extrusion path
        //
        ads_printf(ACRX_T("\nExtrusion path: "));
        getPath(vertices, vertexData, iVertices);
        if (vertices == NULL)
        {
            ads_printf(ACRX_T("\nBad extrusion path.  *Bailing out*\n"));
            goto cleanup;
        }

        ads_printf(ACRX_T("\nClose Path? "));
        bClosePath = getYesNo(FALSE);

        // Get scale factor and twist angle
        //
        scaleFactor = getReal(ACRX_T("\nScale factor <1.0>: "), 1.0);
        twistAngle = getReal(ACRX_T("\nTwist angle <0.0>: "), 0.0);
        if (scaleFactor != 1.0 || twistAngle != 0.0)
        {
            try {
                fixedPt = getPoint(ACRX_T("Select fixed point for scale/twist <0,0,0>: "));
            }
            catch (int caught_adsrc)
            {
                if (caught_adsrc)
                    fixedPt = Point3d::kNull;
                else
                    throw caught_adsrc;
            }
        }

        ads_printf(ACRX_T("\nCheck validity of result? "));
        bCheckValidity = getYesNo();

        pBody = new AsdkBody;

        // The two const_casts are necessary. We need to make them const,
        // so we call the right verson of body() which calls assertReadEnabled().
        // pStartFace and pEndFace are currently open for read.
        //
        pBody->createExtrusionAlongPath(
            const_cast<const AsdkBody*>(pStartFace)->body(),
            const_cast<const AsdkBody*>(pEndFace)->body(),
            vertices, 
            vertexData, 
            iVertices, 
            bClosePath,
            bCheckValidity,
            fixedPt,
            (double)scaleFactor, 
            (double)twistAngle, 
            *pMorphMap);
    }
    catch (int caught_adsrc)
    //
    // User probably cancelled.  Clean up and bail out.
    {
        handleADSError(caught_adsrc);
        goto cleanup;
    }
	catch (ErrorCode err) 
    {
        if (err == eFail)
        {
            ads_printf( ACRX_T("\nBody is not valid.  Continue?"));
            if(! getYesNo())
            {
                delete pBody;
                goto cleanup;
            }
        }
        else
        {
            ads_printf( ACRX_T("\nERROR: %s\n"), getErrorMsg(err));
            delete pBody;
            goto cleanup;
        }
    }
    
    if (append(pBody))
        pBody->close();
    else {
        delete pBody;
        ads_printf(ACRX_T("\nCould not add AsdkBody to ACAD Database.  *Bailing out*\n"));
    }

cleanup:

    cleanupEntity(pStartFace);
    cleanupEntity(pEndFace);
    delete [] vertices;
    delete [] vertexData;
}


void 
aUnion()
{
    ads_name s;
    if (RTNORM != ads_ssget( NULL, NULL, NULL, NULL, s ))
        return;

    ads_printf( ACRX_T("\n") );

    int len = 0;
    ads_sslength( s, &len );
    if (len < 2) {
        ads_printf( ACRX_T("Nothing to do.\n") );
        ads_ssfree( s );
        return;
    }

    AsdkBody *pFirst = NULL;
    ads_name first_ent;
    for (long i = 0; i < len; i++)
    {
        ads_name ent;
        AcDbObjectId id;

        if (RTNORM != ads_ssname( s, i, ent ))
            continue;
        if (Acad::eOk != acdbGetObjectId( id, ent ))
            continue;

        AsdkBody *p;
        if (Acad::eOk != acdbOpenObject( p, id, AcDb::kForWrite ))
            continue;

        if (NULL == pFirst)
        {
            pFirst = p;
            ads_name_set(ent, first_ent);
            //
            // Not sure if this is the right way to copy an ads_name -vm
        }
        else 
        {
            try 
            {
                pFirst->body() += p->body();
                p->erase();
            }
            catch (...) 
            {
                ads_printf( ACRX_T("*Invalid*\n") );
            }
            p->close();
        }
    }
    
    ads_ssfree( s );
    if (NULL != pFirst)
    {
        pFirst->close();
        ads_entupd(first_ent);
    }
}


void 
aSubtract()
{
    ads_name ent1, ent2;
    ads_point p1, p2;

    if (RTNORM != ads_entsel( ACRX_T("\nSelect AsdkBody to subtract from: "), ent1, p1 ))
    {
        ads_printf( ACRX_T("*Cancel*\n") );
        return;
    }
    if (RTNORM != ads_entsel( ACRX_T("\nSelect AsdkBody to subtract: "), ent2, p2 ))
    {
        ads_printf( ACRX_T("*Cancel*\n") );
        return;
    }
    ads_printf( ACRX_T("\n") );

    AcDbObjectId id1, id2;
    if (Acad::eOk != acdbGetObjectId( id1, ent1 ))
    {
        ads_printf( ACRX_T("*Cancel*\n") );
        return;
    }
    if (Acad::eOk != acdbGetObjectId( id2, ent2 ))
    {
        ads_printf( ACRX_T("*Cancel*\n") );
        return;
    }

    AsdkBody *pb1, *pb2;
    if (Acad::eOk != acdbOpenObject( pb1, id1, AcDb::kForWrite ))
    {
        ads_printf( ACRX_T("*Cancel*\n") );
        return;
    }
    if (Acad::eOk != acdbOpenObject( pb2, id2, AcDb::kForWrite ))
    {
        pb1->close();
        ads_printf( ACRX_T("*Cancel*\n") );
        return;
    }

    try {
        pb1->body() -= pb2->body();
        pb2->erase();
    }
    catch (...) {
        ads_printf( ACRX_T("*Invalid*\n") );
    }
    pb1->close();
    pb2->close();

    ads_entupd(ent1);
}


void 
aIntersect()
{
    ads_name s;
    if (RTNORM != ads_ssget( NULL, NULL, NULL, NULL, s ))
        return;

    ads_printf( ACRX_T("\n") );

    int len = 0;
    ads_sslength( s, &len );
    if (len < 2) 
    {
        ads_printf( ACRX_T("Nothing to do.\n") );
        ads_ssfree( s );
        return;
    }

    AsdkBody *pFirst = NULL;
    ads_name first_ent;
    for (long i = 0; i < len; i++)
    {
        ads_name ent;
        AcDbObjectId id;
        if (RTNORM != ads_ssname( s, i, ent ))
            continue;
        if (Acad::eOk != acdbGetObjectId( id, ent ))
            continue;

        AsdkBody *p;
        if (Acad::eOk != acdbOpenObject( p, id, AcDb::kForWrite ))
            continue;

        if (NULL == pFirst)
        {
            pFirst = p;
            ads_name_set(ent, first_ent);
        }
        else 
        {
            try 
            {
                pFirst->body() *= p->body();
                p->erase();
            }
            catch (...) 
            {
                ads_printf( ACRX_T("*Invalid*\n") );
            }
            p->close();
        }
    }
    ads_ssfree( s );
    if (NULL != pFirst) 
    {
        if (pFirst->body().isNull()) 
        {
            ads_printf( ACRX_T("Null AsdkBody created - deleted\n") );
            pFirst->erase();
        }
        pFirst->close();
        ads_entupd(first_ent);
    }
}


//
// Only for profiling purposes.
//
void
doArray()
{
    acedCommandS(
        RTSTR, ACRX_T("_array"),
        RTSTR, ACRX_T("_l"),
        RTSTR, ACRX_T(""),
        RTSTR, ACRX_T("_r"),
        RTSTR, ACRX_T("30"),
        RTSTR, ACRX_T("30"),
        RTSTR, ACRX_T("0,0"),
        RTSTR, ACRX_T("15,15"),
        NULL );
}


////////////////////////////////////////////////////////////////////////////////
//
// Face and morphing map creation functions.
//
// These functions are specific to Facet Modeler objects.
//
// Each throws an ADS return code if the user cancels out of an operation,
// or there is an unanticipated error.
//

#define STREQ(s,t) (_tcscmp((s),(t))==0)

//
// getFace()
//
// Prompts the user to specify a face in one of three ways:
// 1) Select an existing AsdkBody that consists of one face,
// 2) Select a polyline to be used as a pattern for the face
//    vertices (the default), or
// 3) Enter the face vertices one-by-one.
//
// A AsdkBody object, opened for reading, is returned.
//
static AsdkBody* 
getFace()
{
	int adsrc;
    ACHAR kw[20];

    // Limit the choices for the time being -vm
    //
    // ads_initget(0, "Points polyLine Face");
	// adsrc = ads_getkword("Points/<polyLine>/Face: ", kw);

    ads_initget(0, ACRX_T("Points polyLine"));
	adsrc = ads_getkword(ACRX_T("Points/<polyLine>: "), kw);
    if (adsrc <= RTERROR) 
		throw adsrc;
    
    if (adsrc == RTNONE) 
    {
        _tcscpy_s(kw, ACRX_T("polyLine"));
		adsrc = RTNORM;
    }

	if (STREQ(kw, ACRX_T("Face")))
        return getExistingFace();
    else
        return getNewFace(kw);
}


//
// getNewFace()
//
// If kw == "polyLine", create a single-face AsdkBody based on an existing polyline.
// If kw == "Points", creat a single-face AsdkBody based on points entered by the user.
// A pointer to the AsdkBody, opened for reading, is returned.
//
static AsdkBody*
getNewFace(ACHAR* kw)
{
    AsdkBody* pBody = NULL;
    AcGePoint3d *vertices = NULL;
    int iVertices;
    AcGeVector3d normal;

    if (STREQ(kw, ACRX_T("polyLine")))
    // Create a new one-face AsdkBody from an exisiting polyline
    //
    {
		PolygonVertexData** vertexData = NULL;
		int iVertices;

		getPolylineVertices(vertices, vertexData, iVertices, normal);

        if (vertices == NULL)
            return NULL;
            
        pBody = new AsdkBody;

        try 
        {
		    pBody->createFace(vertices, vertexData, iVertices, normal);
		}
		catch (ErrorCode err) 
        {
			ads_printf( ACRX_T("\nERROR: %s\n"), getErrorMsg(err));
			delete pBody;
            delete [] vertices;
            delete [] vertexData;
			return NULL;
		}
        delete [] vertexData;
    }
    else if (STREQ(kw, ACRX_T("Points")))
    {
        getVertices(vertices, iVertices);

        if (vertices == NULL)
            return NULL;

		pBody = new AsdkBody;

        try
        {
		    pBody->createFace(vertices, iVertices);
		}
		catch (ErrorCode err) 
        {
		    ads_printf( ACRX_T("\nERROR: %s\n"), getErrorMsg(err));
			delete pBody;
            delete [] vertices;
		    return NULL;
	    }

    }
    else // should never happen
        return NULL;
	
    delete [] vertices;

	if (! append(pBody)) 
    {
		ads_printf( ACRX_T("\nCould not add AsdkBody to ACAD Database.  *Bailing out*\n") );
        delete pBody;
	    return NULL;
	}

	ads_printf(ACRX_T("\nCreated %d edge face."), pBody->body().edgeCount());
	pBody->downgradeOpen();

	return pBody;
}


//
// getExistingFace()
// 
// Prompts the user to select an entity.  If it is a single-face AsdkBody, a
// pointer to the AsdkBody, opened for reading, is returned, otherwise, NULL.
//
static AsdkBody*
getExistingFace()
{
    AsdkBody* pBody = NULL;
	AcDbEntity* pEnt = NULL;
    
    ads_printf(ACRX_T("\nSelect a single-face AsdkBody :"));
    pEnt = selectEntity(AcDb::kForRead);

	if (pEnt == NULL) 
    {
		ads_printf( ACRX_T("No entity selected.\n") );
		ads_printf( ACRX_T("*Cancel*\n") );
		return NULL;
	}

	pBody = AsdkBody::cast(pEnt);

	if (pBody == NULL) 
    {
		ads_printf(ACRX_T("\nEntity is not a AsdkBody object.  *Bailing out*\n"));
		pEnt->close();
		return NULL;
	}
    if (pBody->body().faceCount() != 1)
    {
        ads_printf(ACRX_T("\nAsdkBody object must have exactly one face. *Bailing out*\n"));
        pBody->close();
        return NULL;
    }
 
    ads_printf(ACRX_T("\nGot a %d edge face."), pBody->body().edgeCount());

    return pBody;
}


//
// getMorphingMap
//
// Prompts the user to enter all of the vertex connections necessary
// to create a morphing map, based on the start and end profile faces
// passed to it.
// 
// The function will iterate on the vertices of the face which has
// the most vertices, assuring completeness.
//
static MorphingMap
getMorphingMap(const Body& startProfile, const Body& endProfile)
{
	int startCount = startProfile.edgeCount();
	int endCount = endProfile.edgeCount();
	
	int vertex;
	MorphingMap morphMap; // returned

    // The defaultVertex is equal to the same index on the opposite
    // face, unless it is greater than the max index on that face,
    // in which case it equals the max index.
    //
    int defaultVertex;
 
    // Not documented what this does, or if I need to do it.
    //
	morphMap.init();

	if (endCount > startCount)
	{
		ads_printf(ACRX_T("\nMorph from end profile to start profile.\n"));
		for(int i = 0; i < endCount; i++) {
			defaultVertex = i > startCount - 1 ? startCount - 1 : i;
			ads_printf(ACRX_T("Map vertex %d to vertex [0-%d] <%d> :"), 
					i, startCount - 1, defaultVertex);
            vertex = getInt(ACRX_T(""), defaultVertex);

			morphMap.add(vertex, i);
		}
	}
	else // startCount >= endCount
	{
		ads_printf(ACRX_T("\nMorph from start profile to end profile.\n"));
		for(int i = 0; i < startCount; i++) {
			defaultVertex = i > endCount - 1 ? endCount - 1 : i;
			ads_printf(ACRX_T("Map vertex %d to vertex [0-%d] <%d> :"), 
					i, endCount - 1, defaultVertex);
			vertex = getInt(ACRX_T(""), defaultVertex);

			morphMap.add(i, vertex);
		}
	}
	return morphMap;
}


//
// queryMorphingMap()
//
// Encapsulates getMorphingMap, performing all the necessary
// user interaction.
//
static MorphingMap*
queryMorphingMap(AsdkBody* pBody1, AsdkBody* pBody2)
{
    bool bGetMorphingMap;
	pBody1->upgradeOpen();
	pBody2->upgradeOpen();
	
    if (pBody1->body().edgeCount() == pBody2->body().edgeCount())
	{
		pBody1->downgradeOpen();
		pBody2->downgradeOpen();
		ads_printf(ACRX_T("\nSpecify Morphing Map? "));
		bGetMorphingMap = getYesNo(FALSE);
	}
	else
	{
		bGetMorphingMap = TRUE;
	}
	
	
	if (bGetMorphingMap)
	{
		pBody1->upgradeOpen();
		pBody2->upgradeOpen();
		MorphingMap* map;
        map = new MorphingMap(getMorphingMap(pBody1->body(), pBody2->body()));
		pBody1->downgradeOpen();
		pBody2->downgradeOpen();
		return map;
	}
    else
        return NULL;
}


////////////////////////////////////////////////////////////////////////////////
//
// ADS utility functions
//
// Each of these encapsulates one or more ADS functions and performs conversion
// to AcGe class objects.
//
// The ADS return code is thrown if the user cancels, or there is an unforseen
// error.
//

//
// getPolyline()
//
// Prompts the user to select an entity, then attempts to cast it
// to a AcDb2dPolyline.  Returns NULL if this doesn't work.
//
static AcDb2dPolyline*
getPolyline()
{
	AcDb2dPolyline* pPLine = NULL;
	AcDbEntity* pEnt = NULL;
    
    while (1)
    {
        ads_printf(ACRX_T("Select polyline :"));
        pEnt = selectEntity(AcDb::kForRead);

        if (pEnt == NULL)
            return NULL;


	    pPLine = AcDb2dPolyline::cast(pEnt);

		if (pPLine == NULL) 
        {
			AcDbPolyline* pLwPline = NULL;

			if (pEnt->isA() != AcDbPolyline::desc())
			{
				ads_printf(ACRX_T("\nEntity must be a polyline.\n"));
				pEnt->close();
				continue;
			}
			else // LWPolyline found, convert
			{
				pEnt->upgradeOpen();
				pLwPline = AcDbPolyline::cast(pEnt);
				Acad::ErrorStatus es;
				pLwPline->upgradeOpen();
				es = pLwPline->convertTo(pPLine);
				if (es == Acad::eObjectToBeDeleted)
				{
					delete(pLwPline);
					pPLine->downgradeOpen();
				}
				else
					return NULL;
				break;
			}
		}
        else
            break;
    }

	return pPLine;
}


static AcGeVector3d
getVector()
{
    AcGePoint3d p1(0,0,0), p2(0,0,0);

    getTwoPoints(p1, p2);
    return p2 - p1;
}


//
// getTwoPoints()
//
// Used to get a line.  When I figure out how to
// convert an AcGeLine3d, I'll would pass back one of
// those, instead.
//
static void
getTwoPoints(AcGePoint3d& p1, AcGePoint3d &p2)
{
    AcGePoint3d *vertices = NULL;
    int iVertices;

    getVertices(vertices, iVertices, 2);
        
    if (iVertices == 2)
    {
       p1 = vertices[0];
       p2 = vertices[1];
    }
    if (iVertices == 1)
    {
       p1 = acdbHostApplicationServices()->workingDatabase()->ucsorg();
       p2 = vertices[0];
    }

    delete [] vertices;
}


//
// circleRadiusFromBulge()
//
// ARX expresses arcs in terms of their "bulges", while the Facet Modeler
// uses circle radii.  The function gets the radius from the bulge.
//
double 
circleRadiusFromBulge(const AcGePoint3d& p1, const AcGePoint3d& p2, double bulge)
{
	return 0.25 * p1.distanceTo(p2) * (bulge + 1 / bulge);  
} 


//
// getPolylineVertices()
//
// Gets a polyline entity from the user, and converts it into an array of
// vertices, vertex data, and a normal, which are passed back through the
// parameters.  If something goes wrong, vertices is set to NULL.
//
// Note that PolygonVertexData is an AModeler class.
//
static void 
getPolylineVertices(AcGePoint3d* &vertices, PolygonVertexData** &vertexData, 
	int &iVertices, AcGeVector3d &normal)
{
	int i;
    int approx = -1;
    double filletRadius;
	AcDb2dPolyline* pPLine = getPolyline();

	if (pPLine == NULL) 
    {
		vertices = NULL;
		vertexData = NULL;
		iVertices = 0;
		return;
	}

	AcDbObjectIterator* pIterator = pPLine->vertexIterator();

	AcDb2dVertex* pVertex;
	AcDbObjectId vId;

    // Loop 1:
    // Count the vertices.
    //
	for (iVertices = 0; ! pIterator->done(); pIterator->step(), iVertices++) 
        {;}

    if (iVertices == 0)  // something is fishy
    {
        vertices = NULL;
        vertexData = NULL;
        return;
    }

	vertices = new AcGePoint3d[iVertices];
	vertexData = new PolygonVertexData*[iVertices];
	double *bulge = new double[iVertices];

    normal = pPLine->normal();

    // Loop 2:
    // Get the coordinates and bulge of each vertex
    //
	for (pIterator->start(), i=0; ! pIterator->done(); pIterator->step(), i++) 
    {
		vId = pIterator->objectId();
		pPLine->openVertex(pVertex, vId, AcDb::kForRead);

		vertices[i] = pPLine->vertexPosition(*pVertex);
		bulge[i] = pVertex->bulge();
	
		pVertex->close();
	}

    // Loop 3:
    // Create the vertexData for the polyline.
    // This is in its own loop because we need to peek at the next vertex each time.
    // 
    filletRadius = getReal(ACRX_T("\nFillet radius <0.0>: "), 0.0);

	for (i = 0; i < iVertices; i++)
	{
		if (bulge[i] != 0.0 && (pPLine->isClosed() || i + 1 < iVertices))
        // i.e., vertex is beginning of an arc
        // AND this is not the last vertex of an open polyline
        {
			if (approx < 0)
			{
                approx = getInt(ACRX_T("\nEnter # of lines to approximate a circle <32>:"), 32);
                //
                // I'm deliberately allowing invalid values here,
                // i.e., numbers below 4
			}
			AcGePoint3d nextVertex = i + 1 < iVertices ? vertices[i + 1] : vertices[0];
            // nextVertex might be the first vertex

			double radius = circleRadiusFromBulge(vertices[i], nextVertex, bulge[i]);
			
            bool leftOfCenter;

			if (bulge[i] > 0 && bulge[i] < 1) 
				leftOfCenter = TRUE;
			else if (bulge[i] < -1)
				leftOfCenter = TRUE;
			else
				leftOfCenter = FALSE;

			Circle3d circle(
                (Point3d)vertices[i], 
                (Point3d)nextVertex, 
                (Vector3d)normal, 
                radius, 
                leftOfCenter);

			vertexData[i] = new PolygonVertexData(PolygonVertexData::kArc3d,
                circle, approx);
        }
        else // vertex is the first of a staight line segment
        {
            if (filletRadius != 0.0)
            {
                if (approx < 0)
                    approx = getInt(ACRX_T("\nEnter # of lines to approximate a circle <32>:"), 32);

                vertexData[i] = new PolygonVertexData(
                    PolygonVertexData::kFilletByRadius,
                    filletRadius,
                    approx);
            }
            else
            {
                vertexData[i] = NULL;
            }
        }
	}

	delete pIterator;
	delete [] bulge;

	// pPLine->upgradeOpen();
	// pPLine->erase();
    //
    // Maybe better for testing if we don't erase this polyline

	pPLine->close();
}


//
// getVertices()
//
// Prompt the user for a series of points to be returned as
// an array of vertices.  vertices gets set to NULL if something
// went wrong.
//
// Sorry, no arcs or fillets allowed yet.
//
static void 
getVertices(AcGePoint3d* &vertices, int &iVertices, int iMaxVertices)
{
	int i;
    AcGePoint3d* p = new AcGePoint3d[iMaxVertices];

    ads_initget(0, NULL);

    // Loop 1:
    // Get points from the user, and assign them to the p array.
    //
	for (i=0; i < iMaxVertices; i++) 
    {
        try
        {
            p[i] = getPoint(ACRX_T("Select point: "));
        }
        catch (int caught_adsrc)
        {
            if (caught_adsrc == RTNONE)
                break;
            else
                throw caught_adsrc;
        }
        ads_printf(ACRX_T("\n"));
	}

	iVertices = i;

	vertices = new AcGePoint3d[iVertices];
	
    // Loop 2:
    // Copy vertices into returned array
    //
	for (i = 0; i < iVertices; i++) 
    {
		vertices[i] = p[i];
	}
    delete [] p;
}


//
// getPath()
//
// Prompts the user to make a path either from an existing polyline, or by
// specifying individual points.
//
// Throws the ADS return code if there is an error, or the user
// cancels.
//
static void 
getPath(AcGePoint3d* &vertices, PolygonVertexData** &vertexData, int &iVertices)
{
	AcGeVector3d normal; // unused
    ACHAR kw[20];
    int adsrc;

    ads_initget(0, ACRX_T("Points polyLine"));
	adsrc = ads_getkword(ACRX_T("Points/<polyLine>: "), kw);
    if (adsrc <= RTERROR) 
		throw adsrc;
    if (adsrc == RTNONE) 
    {
		_tcscpy_s(kw, ACRX_T("polyLine"));
        adsrc = RTNORM;
    }

	if (STREQ(kw, ACRX_T("Points")))
        getVertices(vertices, iVertices);
    else
        getPolylineVertices(vertices, vertexData, iVertices, normal);
}

//
// selectEntity()
//
// Prompt the user to select an entity.  Returns NULL if nothing is selected.
//
static AcDbEntity*
selectEntity(AcDb::OpenMode openMode)
{
    ads_name en = { 0, 0 };
    ads_point pt = { 0.0, 0.0, 0.0 };  // Initialization experiment  -vm
    int adsrc;
    AcDbObjectId eId;

	ads_initget(0, NULL);

    adsrc = ads_entsel(ACRX_T(""), en, pt);

    if (adsrc == RTERROR)
    {
        adsrc = RTNONE;
    }

    if (adsrc <= RTERROR)
    {
        throw adsrc;
    }

	if (adsrc == RTNONE) 
	{
		ads_printf(ACRX_T("\nNothing Selected."));
        return NULL;
	}

    // Now, exchange the old-fangled ads_name for the new-fangled
    // object id.
    Acad::ErrorStatus retStat = acdbGetObjectId(eId, en);
    if (retStat != Acad::eOk) 
	{
        ads_printf(ACRX_T("\nacdbGetObjectId failed"));
        ads_printf(ACRX_T("\nen==(%lx,%lx), retStat==%d\n"), en[0], en[1], eId);
        return NULL;
    }

    AcDbEntity *pEntObj = NULL;

    if ((retStat = acdbOpenObject(pEntObj, eId, openMode)) != Acad::eOk) 
	{
        ads_printf(
            ACRX_T("\nacdbOpenObject failed: ename=(%lx,%lx), mode==%d retStat==%d\n"),
            en[0], en[1], openMode, retStat);
        return NULL;
    }
    return pEntObj;
}


//
// getYesNo
//
// Prompts the user to enter Yes or No.  The default is determined
// by the argument "defaultIsYes".
//
static bool 
getYesNo(bool defaultIsYes)
{
	int adsrc;
    ACHAR kw[20];
    ads_initget(0, ACRX_T("Yes No"));

	if(defaultIsYes)
		adsrc = ads_getkword(ACRX_T("<Yes>/No: "), kw);
	else
		adsrc = ads_getkword(ACRX_T("Yes/<No>: "), kw);

    if (adsrc <= RTERROR)
    {
        throw adsrc;
    }

    if (adsrc == RTNONE) {
		if(defaultIsYes)
			_tcscpy_s(kw, ACRX_T("Yes"));
		else
			_tcscpy_s(kw, ACRX_T("No"));
        adsrc = RTNORM;
    }

	if (STREQ(kw, ACRX_T("Yes")))
		return TRUE;
	else
		return FALSE;
}


//
// getPoint()
//
// Note that this throws RTNONE!
//
static AcGePoint3d
getPoint(ACHAR *prompt)
{
    int adsrc;
    ads_point p;

    ads_initget( 0, NULL );
    adsrc = ads_getpoint(NULL, prompt, p);
    if (adsrc <= RTERROR)
        throw adsrc;
    if (adsrc == RTNONE)
        throw adsrc; // catch this!
    if (adsrc == RTNORM)
    {
        acdbUcs2Wcs(p, p, Adesk::kFalse);
        return asPnt3d(p);
    }
    
    // if we get this far, "throw" our hands up...
    // OH, HA HA HA! GET IT?!?!
    throw adsrc;

    // To shut up the compiler...
    return AcGePoint3d(0,0,0);  // WILL NEVER DO THIS
}


static double
getDistance(AcGePoint3d fromPt, ACHAR *prompt, double defaultDist)
{
    int adsrc;
    ads_point p;
    ads_real dist = (ads_real)defaultDist;

    ads_initget( 0, NULL );
    acdbWcs2Ucs(asDblArray(fromPt), p, Adesk::kFalse);  
    adsrc = ads_getdist(p, prompt, &dist);
    if (adsrc <= RTERROR)
       throw adsrc;
    
    return (double)dist;
}

static int
getInt(ACHAR *prompt, int defaultValue)
{
    int i;

    ads_initget( 0, NULL );
    int adsrc = ads_getint(prompt, &i);
    if (adsrc <= RTERROR)
        throw adsrc;
	if (adsrc == RTNONE)
        i = defaultValue;

    return i;
}


static double
getReal(ACHAR *prompt, double defaultValue)
{
    double dbl;

    ads_initget( 0, NULL );
    int adsrc = ads_getreal(prompt, &dbl);
    if (adsrc <= RTERROR)
        throw adsrc;
    if (adsrc == RTNONE)
	    dbl = defaultValue;

    return dbl;
}

//
// cleanupEntity()
//
// Erase and close the entity passed.  I'm not sure what else to
// do to eliminate the enitity from the database.  -vm
//
static void
cleanupEntity(AcDbEntity *pEnt)
{
    if (pEnt != NULL)
    {
        if (! pEnt->isWriteEnabled())
            pEnt->upgradeOpen();

        pEnt->erase();
        pEnt->close();
    }
}

static void
handleADSError(int adsrc)
{
    if (adsrc == RTNONE)
        ads_printf(ACRX_T("\nNothing selected.  *Bailing out*\n"));
    else if (adsrc != RTCAN)
        ads_printf(ACRX_T("\nADS error %d. *Bailing out*\n"), adsrc);
}

#include <iostream>
#include <fstream>

void test20256(Body* body, std::string filePath)
{
    AcGePoint3dArray pts;
    std::vector<std::vector<int>> faces;
    std::unordered_map<Vertex*, int> vertexMap;
    int vertextIndex = 1;
    for (Vertex* v = body->vertexList(); v; v = v->next())
    {
        pts.append(v->point());
        vertexMap[v] = vertextIndex++;
    }
    for (Face* f = body->faceList(); f != NULL; f = f->next()) {
        std::vector<int> faceindex;
        Edge* curEdge = f->edgeLoop();
        do {
            Vertex* v = curEdge->vertex();
            faceindex.push_back(vertexMap[v]);
            curEdge = curEdge->next();
        } while (curEdge != f->edgeLoop());
        faces.push_back(faceindex);
    }
    std::ofstream file(filePath);
    if (!file.is_open())
    {
        return;
    }
    for (auto& v : pts) {
        file << "v " << v.x << " " << v.y << " " << v.z << "\n";
    }
    for (auto& f : faces) {
        file << "f";
        for (int& index : f) {
            file << " " << index;
        }
        file << "\n";
    }
    file.close();
}

void toObj() {
    AcDbEntity* pEnt = selectEntity(AcDb::kForWrite);
    if (!pEnt) {
        return;
    }
    AsdkBody* body = (AsdkBody*)pEnt;
    body->body().triangulateAllFaces();
    test20256(&body->body(), std::string("C:\\新建文件夹\\inputs\\inputs\\checker_2.obj"));
    pEnt->close();
}

#include <chrono>
#include <set>

struct Timer
{
    double report() const
    {
        auto now = std::chrono::steady_clock::now();
        return std::chrono::duration<double>(now - startTime).count();
    }

private:
    std::chrono::steady_clock::time_point startTime = std::chrono::steady_clock::now();
};

AcDbEntity* getAsdkBody()
{
    ads_name ssname;
    acedSSGet(nullptr, nullptr, nullptr, nullptr, ssname);
    int len{};
    acedSSLength(ssname, &len);
    if (len != 1)
    {
        acutPrintf(L"\n只能选择一个");
        return nullptr;
    }

    ads_name firstEnt;
    acedSSName(ssname, 0, firstEnt);
    AcDbObjectId firstId;
    acdbGetObjectId(firstId, firstEnt);

    AcDbEntity* pFirstEnt{};
    if (Acad::eOk != acdbOpenObject(pFirstEnt, firstId, AcDb::kForWrite))
    {
        acedSSFree(ssname);
        return nullptr;
    }

    acedSSFree(ssname);
    return pFirstEnt;
}

Acad::ErrorStatus addToModelSpace(AcDbObjectId& objId, AcDbEntity* pEntity)
{
    if (!pEntity)
    {
        return Acad::eNullObjectPointer;
    }

    AcDbBlockTable* pBlockTable = nullptr;
    AcDbBlockTableRecord* pSpaceRecord = nullptr;

    acdbHostApplicationServices()->workingDatabase()->getSymbolTable(pBlockTable, AcDb::kForRead);

    pBlockTable->getAt(ACDB_MODEL_SPACE, pSpaceRecord, AcDb::kForWrite);

    pBlockTable->close();

    pSpaceRecord->appendAcDbEntity(objId, pEntity);

    pEntity->close();
    pSpaceRecord->close();

    return Acad::eOk;
}

void doSubstract()
{
    auto del_ent = [](AcDbEntity* pEnt)
        {
            if (pEnt)
                pEnt->close();
        };

    using EntPtr = std::unique_ptr<AcDbEntity, decltype(del_ent)>;

    auto pFirstBody = getAsdkBody();
    EntPtr __tmp1(nullptr, del_ent);
    __tmp1.reset(pFirstBody);

    if (!pFirstBody || !pFirstBody->isKindOf(AsdkBody::desc()))
    {
        return;
    }

    auto pSecondBody = getAsdkBody();
    EntPtr __tmp2(nullptr, del_ent);
    __tmp2.reset(pSecondBody);
    if (!pSecondBody || !pFirstBody->isKindOf(AsdkBody::desc()))
    {
        return;
    }

    AsdkBody* pThisBody = AsdkBody::cast(pFirstBody);
    AsdkBody* pThatBody = AsdkBody::cast(pSecondBody);

    auto thisBody = pThisBody->body().copy();
    auto thatBody = pThatBody->body().copy();

    Timer timer;
    auto resBody = thisBody.boolOper(thatBody, AModeler::kBoolOperSubtract);
    auto useTime = timer.report();
    acutPrintf(L"\n布尔操作耗时: %5.10lf", useTime);
    AsdkBody* pT1hisBody = new AsdkBody();
    pT1hisBody->body() = resBody;
    AcDbObjectId id;
    addToModelSpace(id, pT1hisBody);
    pFirstBody->close();
    pSecondBody->close();
}

struct edge {
    AcGePoint3d start;
    AcGePoint3d end;
    edge(AcGePoint3d& p0, AcGePoint3d& p1) : start(p0), end(p1) {};
    edge(){}
};

#include <limits>

double maxDouble = std::numeric_limits<double>::max();
double minDouble = -std::numeric_limits<double>::max();

enum faceStatus
{
    ainb,
    bina,
    aoutb,
    bouta
};

struct face {
    faceStatus status;
    bool bCoincide = false;
    AcArray<AcGeIntArray> loops;
    AcGeVector3d normal;
    AcGePoint3dArray pts;
    AcGePoint3d min = AcGePoint3d(maxDouble, maxDouble, maxDouble);
    AcGePoint3d max = AcGePoint3d(minDouble, minDouble, minDouble);;
    void caculateBox() {
        for (auto& p : pts) {
            min.x = std::min(min.x, p.x);
            min.y = std::min(min.y, p.y);
            min.z = std::min(min.z, p.z);
            max.x = std::max(max.x, p.x);
            max.y = std::max(max.y, p.y);
            max.z = std::max(max.z, p.z);
        }
    }
    void reverse() {
        for (auto& loop : loops) {
            for (int i = 0; i < loop.length() / 2; i++) {
                std::swap(loop[i], loop[loop.length() - 1 - i]);
            }
        }
    }
    face(){}
    face(const face& other) {
        normal = other.normal;
        pts = other.pts;
        loops = other.loops;
        min = other.min;
        max = other.max;
    }
};

struct body2 {
    AcArray<AcGeIntArray> faces;
    std::vector<AcGePoint3d> pts;
    AcArray<faceStatus> fStatus;
    AcArray<bool> coincides;
};

struct body {
    AcArray<face*> faces;
    AcGePoint3d min = AcGePoint3d(maxDouble, maxDouble, maxDouble);
    AcGePoint3d max = AcGePoint3d(minDouble, minDouble, minDouble);
    void caculateBox() {
        for (auto& f : faces) {
            min.x = std::min(min.x, f->min.x);
            min.y = std::min(min.y, f->min.y);
            min.z = std::min(min.z, f->min.z);
            max.x = std::max(max.x, f->max.x);
            max.y = std::max(max.y, f->max.y);
            max.z = std::max(max.z, f->max.z);
        }
    }
};

enum Edgelocate {
    out = -1,
    in = 1,
    on = 0
};

enum intersectStatus {
    change = 1,
    unknow = 0,
    unchange = -1
};

struct intersectPt
{
    intersectStatus status;
    AcGePoint3d pt;
    double param;
};

void test(AcGeLine3d& l, std::vector<intersectPt>& arr, face& fa) {
    AcGePoint3d linePt = l.pointOnLine();
    AcGeVector3d lineDir = l.direction();
    for (auto& loop : fa.loops) {
        AcGeDoubleArray params(loop.length());
        AcArray<bool> isOns(loop.length());
        AcGeVector3dArray vs(loop.length());
        AcGeVector3dArray vsCross(loop.length());
        AcGeDoubleArray vsCrossLength(loop.length());
        for (int i = 0; i < loop.length(); i++) {
            vs.append(fa.pts[loop[i]] - linePt);
            vsCross.append(vs.last().crossProduct(lineDir));
            vsCrossLength.append(vsCross.last().length());
            params.append((vs.last()).dotProduct(lineDir));
            if ((linePt + lineDir * params.last()).isEqualTo(fa.pts[loop[i]])) {
                isOns.append(true);
            }
            else {
                isOns.append(false);
            }
        }

        for (int i = 0; i < loop.length(); i++) {
            int prev = i - 1;
            if (i == 0) {
                prev = loop.length() - 1;
            }
            int next = i + 1;
            if (i == loop.length() - 1) {
                next = 0;
            }
            if (isOns[i]) {//起点交点
                if (isOns[next] || isOns[prev]) {//尾部交点，当前段或前段为重合段
                    arr.push_back({ unknow ,fa.pts[loop[i]], params[i] });
                }
                else if (vsCross[i].dotProduct(vsCross[prev]) > 0) {//方向一致
                    arr.push_back({ unchange ,fa.pts[loop[i]], params[i] });
                }
                else {
                    arr.push_back({ change ,fa.pts[loop[i]], params[i] });
                }
            }
            else if(!isOns[next]){//可能有一个内部交点
                if (vsCross[i].dotProduct(vsCross[next]) < 0) {//有内部交点，两个端点分布在两侧
                    //计算内部交点
                    if (vsCrossLength[i] + vsCrossLength[next] < 1e-10) {
                        arr.push_back({ change ,linePt, 0 });
                    }
                    else {
                        double param = vs[i].crossProduct(vs[next]).length() / (vsCrossLength[i] + vsCrossLength[next]);
                        if ((vs[i] + vs[next]).dotProduct(lineDir) > 0) {
                            arr.push_back({ change ,linePt + param * lineDir, param });
                        }
                        else {
                            arr.push_back({ change ,linePt - param * lineDir, -param });
                        }
                        
                    }
                }
            }
        }
    }
}

void mergeTwoArr(AcGeDoubleArray& arr1, AcGeDoubleArray& arr2, AcGeDoubleArray& ret) {
    int i = 0, j = 0;
    while (i < arr1.length() * 2 && j < arr2.length() * 2) {
        double start = std::max(arr1[i], arr2[j]);
        double end = std::min(arr1[i + 1], arr2[j + 1]);
        if (start < end) {
            if (!ret.isEmpty() && start <= ret.last()) {
                ret.last() = std::max(ret.last(), end);
            }
            else {
                ret.append(start);
                ret.append(end);
            }
        }
        if (arr1[i + 1] < arr2[j + 1]) {
            i++;
        }
        else {
            j++;
        }
    }
}

AcArray<edge> planefaceIntersect(AcGePlane& pa, face& fb) {
    AcGePlane pb(fb.pts[fb.loops[0][0]], fb.normal);
    AcGeLine3d l;
    AcArray<edge> edgeRet;
    if (!pa.intersectWith(pb, l)) {
        return edgeRet;
    }
    AcGePoint3d p0 = l.evalPoint(1);
    AcGePoint3d p1 = l.evalPoint(2);
    std::vector<intersectPt> arr2;
    test(l, arr2, fb);
    std::sort(arr2.begin(), arr2.end(), [](intersectPt& i, intersectPt& j) {return i.param < j.param; });
    Edgelocate lastLocate = out;
    double lastParam = 0.0;
    for (int i = 0; i < (int)arr2.size() - 1; i++) {
        if (lastLocate == out) {
            if (arr2[i].status != unchange) {
                //edgeRet.append(edge(arr2[i].pt, arr2[i + 1].pt));
                if (arr2[i + 1].param - arr2[i].param > 1e-10) {
                    if (edgeRet.length() > 0 && arr2[i].param - lastParam < 1e-10) {
                        edgeRet.last().end = arr2[i + 1].pt;
                    }
                    else {
                        edgeRet.append(edge(arr2[i].pt, arr2[i + 1].pt));
                    }
                }
                lastParam = arr2[i + 1].param;
                lastLocate = in;
            }
            else {
                lastLocate = out;
            }
        }
        else if (lastLocate == in) {
            if (arr2[i].status != change) {
                //edgeRet.append(edge(arr2[i].pt, arr2[i + 1].pt));
                if (arr2[i + 1].param - arr2[i].param > 1e-10) {
                    if (edgeRet.length() > 0 && arr2[i].param - lastParam < 1e-10) {
                        edgeRet.last().end = arr2[i + 1].pt;
                    }
                    else {
                        edgeRet.append(edge(arr2[i].pt, arr2[i + 1].pt));
                    }
                }
                lastParam = arr2[i + 1].param;
                lastLocate = in;
            }
            else {
                lastLocate = out;
            }
        }  
    }
    return edgeRet;
}

AcArray<AcArray<edge>> detectLoop(std::list<edge>& edges) {
    AcArray<AcArray<edge>> rets;
    AcArray<edge> ret;
    while (edges.size() > 0) {
        if (ret.length() == 0) {
            ret.append(*edges.begin());
            edges.pop_front();
        }
        if (ret.length() > 2 && ret[0].start.isEqualTo(ret.last().end)) {
            rets.append(ret);
            ret.setLogicalLength(0);
            continue;
        }
        bool find = false;
        for (std::list<edge>::iterator start = edges.begin(); start != edges.end(); ) {
            if ((*start).start.isEqualTo(ret.last().end)) {
                ret.append(*start);
                edges.erase(start);
                find = true;
                break;
            }
            start++;
        }
        if (!find) {
            ret.setLogicalLength(0);//未成环
        }
        if (edges.size() == 0 && ret.length() > 2) {
            rets.append(ret);
        }
    }
    return rets;
}

struct intersectInfo {
    AcGePoint3d pt;
    bool bOut;
    double p1;
    double p2;
    intersectInfo() {};
    intersectInfo(double p1, double p2, bool bOut, AcGePoint3d& pt) {
        this->bOut = bOut;
        this->p1 = p1;
        this->p2 = p2;
        this->pt = pt;
    }
};

AcArray<face*> loopsToFaces(AcArray<AcGePoint3dArray>& loops, AcGeVector3d& normal);

void faceToPts(face& f, AcGePoint3dArray& pts, bool bReverse) {
    for (auto& loop : f.loops) {
        for (int i = 0; i < loop.length(); i++) {
            pts.append(f.pts[loop[bReverse ? loop.length() - i - 1 : i]]);
        }
    }
}

void invertArr(AcGeIntArray& arr, AcGeIntArray& ret) {
    ret.setLogicalLength(arr.length());
    for (int i = 0; i < arr.length(); i++) {
        ret[arr[i]] = i;
    }
}
//采用射线求交
bool ptInLoop(const AcGePoint3d& pt, AcGeVector3d& dir, AcGePoint3dArray& pts) {
    AcGeVector3dArray vs(pts.length());
    for (int i = 0; i < pts.length(); i++) {
        vs.append(pts[i] - pt);
    }
    int intersectCount = 0;
    for (int i = 0; i < vs.length(); i++) {
        const AcGeVector3d& v0 = vs[i];
        const AcGeVector3d& v1 = (i == vs.length() - 1) ? vs[0] : vs[i + 1];
        const AcGeVector3d u1 = dir.crossProduct(v0);
        const AcGeVector3d u2 = dir.crossProduct(v1);
        if (u1.dotProduct(u2) > 0.0) {
            continue;
        }
        AcGePoint3d midPt;
        for (int coord = 0; coord < 3; coord++) {
            double x0 = pts[i][coord];
            double x1;
            if (i == vs.length() - 1) {
                x1 = pts[0][coord];
            }
            else
            {
                x1 = pts[i + 1][coord];
            }
            midPt[coord] = (x0 + x1) / 2.0;
            //midPt[coord] = (pts[i][coord] + (i == vs.length() - 1) ? pts[0][coord] : pts[i + 1][coord]) / 2.0;
        }
        if ((midPt - pt).dotProduct(dir) > 0.0) {
            intersectCount++;
        }
    }
    return intersectCount % 2 == 1;
}

AcArray<face*> facefaceWithNoIntersectPts(face& fa, face& fb, AcGePoint3dArray& faPts, AcGePoint3dArray& fbPts, bool bInner) {
    AcArray<face*> ret;
    if (fa.min.x < fb.min.x && fa.max.x > fb.max.x && fa.min.y < fb.min.y && fa.max.y > fb.max.y && fa.min.z < fb.min.z && fa.max.z > fb.max.z) {
        //fa可能包含fb
        if (ptInLoop(fb.pts[0], fb.normal.perpVector(), faPts)) {
            if (bInner) {
                ret.append(new face(fb));
            }
            else {
                //fa加上fb的反向
                face* temp = new face(fa);
                int ptsLen = temp->pts.length();
                temp->pts.append(fb.pts);
                AcGeIntArray loop = fb.loops[0];
                for (int i = 0; i < loop.length() / 2; i++) {
                    std::swap(loop[i], loop[loop.length() - 1 - i]);
                }
                for (int i = 0; i < loop.length(); i++) {
                    loop[i] += ptsLen;
                }
                temp->loops.append(loop);
                ret.append(temp);
            }
        }
    }
    if (fa.min.x > fb.min.x && fa.max.x < fb.max.x && fa.min.y > fb.min.y && fa.max.y < fb.max.y && fa.min.z > fb.min.z && fa.max.z < fb.max.z) {
        if (ptInLoop(fa.pts[0], fa.normal.perpVector(), fbPts)) {
            if (bInner) {
                ret.append(new face(fa));
            }
        }
    }
    return ret;
}

void dealWithCoLine(AcGePoint3d& p0,AcGeVector3d& vq0p0, AcGeVector3d& vq0p1, AcGeVector3d& vp0q1,AcGePoint3d& q0,
    AcArray<intersectInfo>& intersection_Point, int i, int j, bool bout, AcGeVector3d& vp, AcGeVector3d& vq)
{
    if (vq0p0.dotProduct(vq0p1) < 0 && vq0p1.lengthSqrd() > 1e-20) {
        //q0为交点
        double pam = vq0p0.length() / vp.length();
        intersection_Point.append(intersectInfo(pam + i, j, bout, q0));
    }
    if (vq0p0.dotProduct(vp0q1) > 0 && vp0q1.lengthSqrd() > 1e-20) {
        //p0为交点
        double pam = vq0p0.length() / vq.length();
        intersection_Point.append(intersectInfo(i, j + pam, bout, p0));
    }
}

AcArray<face*> facefaceIntersect(face& fa, face& fb, bool bInner) {//fa为主多边形，fb为裁剪多边形。内裁剪
    //fa，fb都为单环,可以为多环
    AcGePoint3dArray datap0;
    AcGePoint3dArray dataq0;
    faceToPts(fa, datap0, false); faceToPts(fb, dataq0, !bInner);
    AcArray<intersectInfo> intersection_Point;
    for (int i = 0; i < datap0.length(); i++) {
        for (int j = 0; j < dataq0.length(); j++) {
            AcGePoint3d& p0 = datap0[i];
            AcGePoint3d& p1 = datap0[(i + 1) % datap0.length()];
            AcGePoint3d& q0 = dataq0[j];
            AcGePoint3d& q1 = dataq0[(j + 1) % dataq0.length()];
            AcGeVector3d vp = p1 - p0;
            AcGeVector3d vp0q1 = q1 - p0;
            AcGeVector3d v1 = vp.crossProduct(q0 - p0);
            AcGeVector3d v2 = vp.crossProduct(q1 - p0);
            if (v1.dotProduct(v2) > 0.0) {
                continue;
            }
            AcGeVector3d vq = q1 - q0;
            AcGeVector3d vq0p0 = p0 - q0;
            AcGeVector3d vq0p1 = p1 - q0;
            AcGeVector3d u1 = vq.crossProduct(p0 - q0);
            AcGeVector3d u2 = vq.crossProduct(p1 - q0);
            if (u1.dotProduct(u2) > 0.0) {
                continue;
            }
            bool bout = true;
            if (u2.dotProduct(fa.normal) > 0) {
                bout = false;
            }
            AcGeVector3d e1e2 = vp.crossProduct(vq);
            if (e1e2.lengthSqrd() < 1e-20) {
                dealWithCoLine(p0, vq0p0, vq0p1, vp0q1, q0, intersection_Point, i, j, bout, vp, vq);
                continue;
            }
            if (v2.lengthSqrd() < 1e-20 && vp0q1.dotProduct(p1 - q1) >= 0.0) {//q1为交点
                continue;
            }
            if (u2.lengthSqrd() < 1e-20 && vq0p1.dotProduct(q1 - p1) >= 0.0) {//p1交点
                continue;
            }
            double pam1 = (p0 - q0).crossProduct(vq).length() / e1e2.length();
            double pam2 = (p0 + pam1 * vp).distanceTo(q0) / vq.length();
            intersection_Point.append(intersectInfo(pam1 + i, pam2 + j, bout, p0 + (p1 - p0) * pam1));
        }
    }
    if (0 == intersection_Point.length()) {
        return facefaceWithNoIntersectPts(fa, fb, datap0, dataq0, bInner);
    }
    //构造两个交点的array,分别在fa和fb排序的
    AcGeIntArray intersection_Pointa;
    int numOfIntsect = intersection_Point.length();
    for (int i = 0; i < numOfIntsect; i++) {
        intersection_Pointa.append(i);
    }
    AcGeIntArray intersection_Pointb = intersection_Pointa;
    std::sort(intersection_Pointa.begin(), intersection_Pointa.end(), [&](const int& a, const int& b) {return intersection_Point[a].p1 < intersection_Point[b].p1; });
    std::sort(intersection_Pointb.begin(), intersection_Pointb.end(), [&](const int& a, const int& b) {return intersection_Point[a].p2 < intersection_Point[b].p2; });
    //维护两个数组
    AcGeIntArray oTob;
    invertArr(intersection_Pointb, oTob);
    AcGeIntArray aTob, bToa;
    for (int i = 0; i < numOfIntsect; i++) {
        aTob.append(oTob[intersection_Pointa[i]]);
    }
    invertArr(aTob, bToa);
    AcGePoint3dArray ret;
    AcArray<AcGePoint3dArray> rets;
    AcArray<bool> visited;//仅对应第一个
    visited.setLogicalLength(numOfIntsect);
    visited.setAll(false);
    int numVisit = 0;
    for (int i = 0; numVisit < visited.length(); i++) {
        if (i >= visited.length()) {
            i = 0;
        }
        if (visited[i]) {
            if (ret.length() > 0) {
                rets.append(ret);
                ret.setLogicalLength(0);
            }
            continue;
        }
        numVisit++;
        visited[i] = true;
        ret.append(intersection_Point[intersection_Pointa[i]].pt);
        if (intersection_Point[intersection_Pointa[i]].bOut) {//对于a，其loop在此点处出b的loop.否则为进入b的loop
            //开始遍历交点序列b
            int iteratorStart = int(floor(intersection_Point[intersection_Pointb[aTob[i]]].p2)) + 1;
            int iteratorEnd = int(floor(intersection_Point[intersection_Pointb[(aTob[i] + 1) % numOfIntsect]].p2));
            if (iteratorStart > iteratorEnd) {
                if (iteratorStart == iteratorEnd + 1 && intersection_Point[intersection_Pointb[aTob[i]]].p2 < intersection_Point[intersection_Pointb[(aTob[i] + 1) % numOfIntsect]].p2) {//两个交点在同一区间
                    i = bToa[(aTob[i] + 1) % numOfIntsect] - 1;
                    continue;
                }
                iteratorEnd += dataq0.length();
            }
            for (int j = iteratorStart; j <= iteratorEnd; j++) {
                ret.append(dataq0[j % dataq0.length()]);
            }
            i = bToa[(aTob[i] + 1) % numOfIntsect] - 1;
            continue;
        }
        else {
            //开始遍历交点序列a
            int iteratorStart = int(floor(intersection_Point[intersection_Pointa[i]].p1)) + 1;
            int iteratorEnd = int(floor(intersection_Point[intersection_Pointa[(i + 1) % numOfIntsect]].p1));
            if (iteratorStart > iteratorEnd) {
                if (iteratorStart == iteratorEnd + 1 && intersection_Point[intersection_Pointa[i]].p1 < intersection_Point[intersection_Pointa[(i + 1) % numOfIntsect]].p1) {//两个交点在同一区间
                    continue;
                }
                iteratorEnd += datap0.length();
            }
            for (int j = iteratorStart; j <= iteratorEnd; j++) {
                ret.append(datap0[j % datap0.length()]);
            }
            continue;
        }
    }
    if (ret.length() > 0) {
        rets.append(ret);
    }
    return loopsToFaces(rets, fa.normal);
}

#include "geblok3d.h"

AcArray<face*> loopsToFaces(AcArray<AcGePoint3dArray>& loops, AcGeVector3d& normal)
{
    AcArray<AcGeBoundBlock3d> blocks;
    blocks.setLogicalLength(loops.length());
    AcGePoint3dArray blockMinPt, blockMaxPt;
    blockMinPt.setLogicalLength(loops.length()), blockMaxPt.setLogicalLength(loops.length());
    AcArray<bool> loopDirct, vistited;
    loopDirct.setLogicalLength(loops.length()); vistited.setLogicalLength(loops.length());
    loopDirct.setAll(true); vistited.setAll(false);
    for (int i = 0; i < loops.length(); i++) {
        AcGeBoundBlock3d& block = blocks[i];
        for (auto& pt : loops[i]) {
            block.extend(pt);
        }
        block.getMinMaxPoints(blockMinPt[i], blockMaxPt[i]);
        AcGeVector3d normaltemp;
        for (int j = 0; j < loops[i].length(); j++) {
            if (j == loops[i].length() - 1) {
                normaltemp += ((AcGeVector3d*)(&(loops[i][j])))->crossProduct(*(AcGeVector3d*)(&(loops[i][0])));
            }
            else {
                normaltemp += ((AcGeVector3d*)(&(loops[i][j])))->crossProduct(*(AcGeVector3d*)(&(loops[i][j + 1])));
            }
        }
        if (normaltemp.dotProduct(normal) < 0) {
            loopDirct[i] = false;
        }
    }
    AcArray<face*> rets;
    for (int i = 0; i < loops.length(); i++) {
        if (!loopDirct[i] || vistited[i]) {
            continue;
        }
        vistited[i] = true;
        AcArray<AcGePoint3dArray> ret;
        ret.append(loops[i]);
        for (int j = 0; j < loops.length(); j++) {
            if (vistited[j] || loopDirct[j]) {
                continue;
            }
            if (blocks[i].contains(blockMinPt[j]) && blocks[i].contains(blockMaxPt[j])) {
                ret.append(loops[j]);
                vistited[j] = true;
            }
        }
        face* f = new face();
        f->normal = normal;
        for (auto& pts : ret) {
            AcGeIntArray loop;
            for (int ptIndex = f->pts.length(); ptIndex < f->pts.length() + pts.length(); ptIndex++) {
                loop.append(ptIndex);
            }
            f->pts.append(pts);
            f->loops.append(loop);
        }
        rets.append(f);
    }
    return rets;
}

AcArray<face*> AinB;
AcArray<face*> AoutB;
AcArray<face*> BinA;
AcArray<face*> BoutA;
template<class T1, class T2>
bool encounter(const T1& t1, const T2& t2) {
    for (int i = 0; i < 3; i++) {
        if (t1.min[i] > t2.max[i] || t1.max[i] < t2.min[i]) {
            return false;
        }
    }
    return true;
}

void faceBodyIntersect(face& fa, body& b, AcArray<face*>& ret, AcArray<face*>& ret2) {
    if (!encounter(fa, b)) {
        ret2.append(&fa);
        return;
    }
    std::list<edge> intersLine;
    AcGePlane pa(fa.pts[fa.loops[0][0]], fa.normal);
    for (auto& fb : b.faces) {
        AcArray<edge> interRet = planefaceIntersect(pa, *fb);
        for (auto& edge : interRet) {
            intersLine.push_back(edge);
        }
    }
    AcArray<AcArray<edge>> loops = detectLoop(intersLine);
    AcArray<AcGePoint3dArray> ptloops(loops.length());
    for (auto& loop : loops) {
        AcGePoint3dArray ptloop(loop.length());
        for (auto& e : loop) {
            ptloop.append(e.start);
        }
        ptloops.append(ptloop);
    }
    AcArray<face*> fbs = loopsToFaces(ptloops, fa.normal);
    for (int i = 0; i < fbs.length(); i++) {
        ret.append(facefaceIntersect(fa, *fbs[i], true));
        ret2.append(facefaceIntersect(fa, *fbs[i], false));
    }
}

enum boolType {
    Union,
    Intersect,
    Subtruct
};

void caculateFace(body2& bd) {
    std::vector<AcGeIntArray> vertexToFace;
    std::vector<std::set<int>> faceToFace;
    AcArray<AcGeIntArray> orderFace = bd.faces;
    vertexToFace.resize(bd.pts.size());
    for (int i = 0; i < bd.faces.length(); i++) {
        for (auto v : bd.faces[i]) {
            vertexToFace[v].append(i);
        }
    }
    for (int i = 0; i < bd.faces.length(); i++) {
        std::set<int> neighboorFace;
        for (auto v : bd.faces[i]) {
            for (auto f : vertexToFace[v]) {
                neighboorFace.emplace(f);
            }
        }
        faceToFace.push_back(neighboorFace);
    }
    for (auto& f : orderFace) {
        std::sort(f.begin(), f.end());
    }
    for (int i = 0; i < bd.faces.length(); i++) {
        for (auto f : faceToFace[i]) {
            if (i == f) {
                continue;
            }
            faceToFace[f].erase(i);
            if (bd.faces[i].length() != bd.faces[f].length()) {
                continue;
            }
            if (orderFace[i] == orderFace[f]) {
                bd.coincides[i] = true;
                bd.coincides[f] = true;
            }
        }
    }
}

void hilbertSort(std::vector<int>& sortedIndex, const std::vector<AcGePoint3d*>& pts);

void compressVertex2(body* bd, body2& ret) {
    std::vector<AcGePoint3d>& pts2 = ret.pts;
    std::vector<AcGePoint3d*> pts;
    std::vector<int> sortedIndex;
    int vtCount = 0;
    for (auto& f : bd->faces) {
        for (auto& pt : f->pts) {
            pts.push_back(&pt);
            sortedIndex.push_back(vtCount++);
        }
    }
    hilbertSort(sortedIndex, pts);
    AcGePoint3dArray ptss;
    for (int i = 0; i < pts.size(); i++) {
        ptss.append(*pts[sortedIndex[i]]);
    }
    pts2.push_back(*pts[sortedIndex[0]]);
    std::vector<int> sortedIndex2 = sortedIndex;
    sortedIndex2[0] = 0;
    for (int i = 1; i < sortedIndex.size(); i++) {
        if (pts2.back().isEqualTo(*pts[sortedIndex[i]])) {
            sortedIndex2[i] = sortedIndex2[i - 1];
        }
        else {
            pts2.push_back(*pts[sortedIndex[i]]);
            sortedIndex2[i] = sortedIndex2[i - 1] + 1;
        }
    }
    AcGeIntArray invertSortedIndex;
    invertSortedIndex.setLogicalLength(int(sortedIndex.size()));
    for (int i = 0; i < sortedIndex.size(); i++) {
        invertSortedIndex[sortedIndex[i]] = i;
    }
    vtCount = 0;
    for (auto& f : bd->faces) {
        AcGeIntArray tempFace;
        tempFace.setLogicalLength(f->loops[0].length());
        for (int i = 0; i < f->loops[0].length(); i++) {
            tempFace[i] = sortedIndex2[invertSortedIndex[f->loops[0][i] + vtCount]];
        }
        vtCount += f->pts.length();
        ret.fStatus.append(f->status);
        ret.faces.append(tempFace);
    }
    ret.coincides.setLogicalLength(ret.faces.length());
    for (int i = 0; i < ret.coincides.length(); i++) {
        ret.coincides[i] = false;
    }
}

void dealWithMultiLoops(face& face) {
    while (face.loops.length() > 1) {
        //求内环的最远点
        AcGeIntArray& innerLoop = face.loops.last();
        AcGeIntArray& outterLoop = face.loops[0];
        AcGePoint3d innerPt;
        for (int i = 0; i < innerLoop.length(); i++) {
            innerPt.x += face.pts[innerLoop[i]].x;
            innerPt.y += face.pts[innerLoop[i]].y;
            innerPt.z += face.pts[innerLoop[i]].z;
        }
        innerPt.x /= innerLoop.length(); innerPt.y /= innerLoop.length(); innerPt.z /= innerLoop.length();
        double maxDist2 = 0;
        int maxPtIndex = -1;
        for (int i = 0; i < innerLoop.length(); i++) {
            double tempDist2 = (face.pts[innerLoop[i]] - innerPt).lengthSqrd();
            if (tempDist2 > maxDist2) {
                maxDist2 = tempDist2;
                maxPtIndex = i;
            }
        }
        //求该点到外环的最近点
        double minDist2 = maxDouble;
        int minPtindex = -1;
        for (int i = 0; i < outterLoop.length(); i++) {
            double tempDist2 = (face.pts[outterLoop[i]] - face.pts[innerLoop[maxPtIndex]]).lengthSqrd();
            if (tempDist2 < minDist2) {
                minDist2 = tempDist2;
                minPtindex = i;
            }
        }
        //添加内环
        AcGeIntArray newOutterLoop;
        for (int i = 0; i <= minPtindex; i++) {
            newOutterLoop.append(outterLoop[i]);
        }
        for (int i = 0; i <= innerLoop.length(); i++) {
            newOutterLoop.append(innerLoop[(maxPtIndex + i) % innerLoop.length()]);
        }
        newOutterLoop.append(outterLoop[minPtindex]);
        for (int i = minPtindex + 1; i < outterLoop.length(); i++) {
            newOutterLoop.append(outterLoop[i]);
        }
        face.loops[0] = newOutterLoop;
        face.loops.removeLast();
    }
}

void bodyBodyBool(body& a, body& b, boolType type, body2& ret) {
    for (auto& f : a.faces) {
        faceBodyIntersect(*f, b, AinB, AoutB);
    }
    for (auto& f : b.faces) {
        faceBodyIntersect(*f, a, BinA, BoutA);
    }
    body temp;
    auto addFaceToTemp = [&](AcArray<face*>& faces, faceStatus status) {
        for (auto& f : faces) {
            f->status = status;
            temp.faces.append(f);
        }
        };
    addFaceToTemp(AinB, ainb);
    addFaceToTemp(AoutB, aoutb);
    addFaceToTemp(BinA, bina);
    addFaceToTemp(BoutA, bouta);
    for (auto& f : temp.faces) {
        dealWithMultiLoops(*f);
    }
    compressVertex2(&temp, ret);
    caculateFace(ret);
    AcArray<AcGeIntArray> faces(ret.faces.length());
    switch (type) {
    case Union:
        for (int i = 0; i < ret.faces.length(); i++) {
            if (ret.fStatus[i] == aoutb || ret.fStatus[i] == bouta || (ret.fStatus[i] == ainb && ret.coincides[i])) {
                faces.append(ret.faces[i]);
            }
        }
        break;
    case Intersect:
        for (int i = 0; i < ret.faces.length(); i++) {
            if (ret.fStatus[i] == ainb || (ret.fStatus[i] == bina && !ret.coincides[i])) {
                faces.append(ret.faces[i]);
            }
        }
        break;
    case Subtruct:
        for (int i = 0; i < ret.faces.length(); i++) {
            if (ret.fStatus[i] == aoutb) {
                faces.append(ret.faces[i]);
            }
            if (ret.fStatus[i] == bina && !ret.coincides[i]){
                faces.append(ret.faces[i]);
                for (int j = 0; j < ret.faces[i].length() / 2; j++) {
                    std::swap(faces.last()[j], faces.last()[ret.faces[i].length() - 1 - j]);
                }
            }
        }
        break;
    default:
        break;
    }
    ret.faces = faces;
}

void BodyTobody(Body* bd, body& body) {
    for (Face* f = bd->faceList(); f != NULL; f = f->next()) {
        face* tempFace = new face();
        tempFace->normal = (f->plane().normal);
        tempFace->loops.setLogicalLength(1);
        std::vector<int> faceindex;
        Edge* curEdge = f->edgeLoop();
        do {
            Vertex* v = curEdge->vertex();
            tempFace->loops[0].append(tempFace->pts.length());
            tempFace->pts.append((v->point()));
            curEdge = curEdge->next();
        } while (curEdge != f->edgeLoop());
        tempFace->caculateBox();
        body.faces.append(tempFace);
    }
    body.caculateBox();
}

struct cmp {
    const std::vector<AcGePoint3d*>& pts;
    int coord;
    bool up;
    bool operator() (int i1, int i2) {
        //if (coord == 0) {
        //    return (pts[i1]->x < pts[i2]->x) == up;
        //}
        //else if (coord == 1) {
        //    return (pts[i1]->y < pts[i2]->y) == up;
        //}
        //else {
        //    return (pts[i1]->z < pts[i2]->z) == up;
        //}
        if (pts[i1]->x < pts[i2]->x - 1e-10) {
            return true;
        }
        if (pts[i1]->x > pts[i2]->x + 1e-10) {
            return false;
        }
        if (pts[i1]->y < pts[i2]->y - 1e-10) {
            return true;
        }
        if (pts[i1]->y > pts[i2]->y + 1e-10) {
            return false;
        }
        if (pts[i1]->z < pts[i2]->z - 1e-10) {
            return true;
        }
        if (pts[i1]->z > pts[i2]->z + 1e-10) {
            return false;
        }
        return true;
    }
};

std::vector<int>::iterator reorder_split(std::vector<int>::iterator b, std::vector<int>::iterator e, cmp cmp) {
    if (b >= e) return b;
    std::vector<int>::iterator m = b + (e - b) / 2;
    std::nth_element(b, m, e, cmp);
    return m;
}

void sort(const std::vector<AcGePoint3d*>& pts, std::vector<int>::iterator b, std::vector<int>::iterator e, int coordx, bool upx, bool upy, bool upz) {
    const int coordy = (coordx + 1) % 3;
    const int coordz = (coordx + 2) % 3;
    if (e - b <= 1) {
        return;
    }
    std::vector<int>::iterator m0 = b;
    std::vector<int>::iterator m8 = e;
    std::vector<int>::iterator m4 = reorder_split(m0, m8, cmp{pts, coordx, upx});
    std::vector<int>::iterator m2 = reorder_split(m0, m4, cmp{pts, coordy, upy});
    std::vector<int>::iterator m1 = reorder_split(m0, m2, cmp{pts, coordz, upz});
    std::vector<int>::iterator m3 = reorder_split(m2, m4, cmp{pts, coordz, !upz});
    std::vector<int>::iterator m6 = reorder_split(m4, m8, cmp{pts, coordy, !upy});
    std::vector<int>::iterator m5 = reorder_split(m4, m6, cmp{pts, coordz, upz});
    std::vector<int>::iterator m7 = reorder_split(m6, m8, cmp{pts, coordz, !upz});
    sort(pts, m0, m1, coordz, upz, upx, upy);
    sort(pts, m1, m2, coordy, upy, upz, upx);
    sort(pts, m2, m3, coordy, upy, upz, upx);
    sort(pts, m3, m4, coordx, upx, !upy, !upz);
    sort(pts, m4, m5, coordx, upx, !upy, !upz);
    sort(pts, m5, m6, coordy, !upy, upz, !upx);
    sort(pts, m6, m7, coordy, !upy, upz, !upx);
    sort(pts, m7, m8, coordz, !upz, !upx, upy);

}

void hilbertSortRecursive(std::vector<int>& sortedIndex, const std::vector<AcGePoint3d*>& pts, std::vector<int>::iterator b, std::vector<int>::iterator e) {
    std::vector<int>::iterator m = b;
    //if ((e - b) > 64) {
    //    m = b + int(0.125 * (e - b));
    //    hilbertSortRecursive(sortedIndex, pts, b, m);
    //}
    sort(pts, m, e, 0, false, false, false);
}

void hilbertSort(std::vector<int>& sortedIndex, const std::vector<AcGePoint3d*>& pts) {
    std::vector<int>::iterator b = sortedIndex.begin();
    std::vector<int>::iterator e = sortedIndex.end();
    hilbertSortRecursive(sortedIndex, pts, b, e);
}

void compressVertex(body* bd, std::vector<AcGePoint3d>& pts2, std::vector<std::vector<int>>& fs) {
    std::vector<AcGePoint3d*> pts;
    std::vector<int> sortedIndex;
    int vtCount = 0;
    for (auto& f : bd->faces) {
        for (auto& pt : f->pts) {
            pts.push_back(&pt);
            sortedIndex.push_back(vtCount++);
        }
    }
    hilbertSort(sortedIndex, pts);
    //std::sort(sortedIndex.begin(), sortedIndex.end(), [&](int i1 ,int i2)
    //    {
    //        if (pts[i1]->x < pts[i2]->x - 1e-10) {
    //            return true;
    //        }
    //        if (pts[i1]->x > pts[i2]->x + 1e-10) {
    //            return false;
    //        }
    //        if (pts[i1]->y < pts[i2]->y - 1e-10) {
    //            return true;
    //        }
    //        if (pts[i1]->y > pts[i2]->y + 1e-10) {
    //            return false;
    //        }
    //        if (pts[i1]->z < pts[i2]->z - 1e-10) {
    //            return true;
    //        }
    //        if (pts[i1]->z > pts[i2]->z + 1e-10) {
    //            return false;
    //        }
    //        return true;
    //    });
    AcGePoint3dArray ptss;
    for (int i = 0; i < pts.size(); i++) {
        ptss.append(*pts[sortedIndex[i]]);
    }
    pts2.push_back(*pts[sortedIndex[0]]);
    std::vector<int> sortedIndex2 = sortedIndex;
    sortedIndex2[0] = 0;
    for (int i = 1; i < sortedIndex.size(); i++) {
        if (pts2.back().isEqualTo(*pts[sortedIndex[i]])) {
            sortedIndex2[i] = sortedIndex2[i - 1];
        }
        else {
            pts2.push_back(*pts[sortedIndex[i]]);
            sortedIndex2[i] = sortedIndex2[i - 1] + 1;
        }
    }
    AcGeIntArray invertSortedIndex;
    invertSortedIndex.setLogicalLength(int(sortedIndex.size()));
    for (int i = 0; i < sortedIndex.size(); i++) {
        invertSortedIndex[sortedIndex[i]] = i;
    }
    vtCount = 0;
    for (auto& f : bd->faces) {
        std::vector<int> tempFace;
        tempFace.resize(f->loops[0].length());
        for (int i = 0; i < f->loops[0].length(); i++) {
            tempFace[i] = sortedIndex2[invertSortedIndex[f->loops[0][i] + vtCount]];
        }
        vtCount += f->pts.length();
        fs.push_back(tempFace);
    }
}

struct noDirEdge
{
    int v1;
    int v2;
    noDirEdge(int v1, int v2) {
        this->v1 = v1;
        this->v2 = v2;
    }
    bool operator<(const noDirEdge& other) const {
        if (v1 + v2 < other.v1 + other.v2)
        {
            return true;
        }
        if (v1 + v2 > other.v1 + other.v2)
        {
            return false;
        }
        return std::min(v1, v2) < std::min(other.v1, other.v2);
    }

};

#include <map>
void bodyToBody(body2* bd, Body& body) {
#ifndef RELEASE
    AcArray<AcGePoint3dArray> ptss;
    for (auto& f : bd->faces) {
        AcGePoint3dArray pts;
        for (auto i : f) {
            pts.append(bd->pts[i]);
        }
        ptss.append(pts);
    }
#endif // !RELEASE
    std::vector<Vertex*> bodyVertex;
    for (auto& p : bd->pts)
    {
        const Point3d ptem(p.x, p.y, p.z);
        Vertex* vertex = new Vertex(ptem, &body);
        bodyVertex.push_back(vertex);
    }

    std::map<noDirEdge, std::vector<Edge*>> edgeMap;
    for (auto& f : bd->faces) {
        int start = f[0];
        Face* face = new Face(&body);
        Edge* firstEdge = nullptr;
        Edge* prevEdge = nullptr;
        for (int i = 0; i < f.length(); i++) {
            int startIndex = f[i];
            int endIndex;
            if (i == f.length() - 1) {
                endIndex = start;
            }
            else {
                endIndex = f[i + 1];
            }
            Vertex* startVertex = bodyVertex[startIndex];
            Vertex* endVertex = bodyVertex[endIndex];
            noDirEdge parterKey = noDirEdge(startIndex, endIndex);
            auto it = edgeMap.find(parterKey);

            Edge* edge = new Edge(startVertex, face, prevEdge, NULL);
            if (it != edgeMap.end())
            {

                it->second.push_back(edge);
            }
            else
            {
                std::vector<Edge*> edges;
                edges.push_back(edge);
                edgeMap[parterKey] = edges;
            }
            if (!firstEdge) {
                firstEdge = edge;
            }
            else {
                prevEdge->setNext(edge);
            }
            prevEdge = edge;
        }
        prevEdge->setNext(firstEdge);
        firstEdge->setPrev(prevEdge);
        face->setEdgeLoop(firstEdge);
    }
    for (auto it : edgeMap)
    {
        std::vector<Edge*> t = it.second;
        if (t.size() == 2)
        {
            if (t[0]->vertex() == t[1]->vertex()) {
                continue;
            }
            t[0]->setPartner(t[1]);
            t[1]->setPartner(t[0]);
        }
        if (t.size() == 4)
        {
            std::vector<Edge*> t0;
            std::vector<Edge*> t1;
            t0.push_back(t[0]);
            for (int i = 1; i < 4; i++) {
                if (t[i]->vertex() == t[0]->vertex()) {
                    t0.push_back(t[i]);
                }
                else {
                    t1.push_back(t[i]);
                }
            }
            if (t0.size() != t1.size()) {
                continue;
            }
            t0[0]->setPartner(t1[0]);
            t1[0]->setPartner(t0[1]);
            t0[1]->setPartner(t1[1]);
            t1[1]->setPartner(t0[0]);

        }
        if (t.size() == 3)
        {
            int ttt = 234;
        }
    }
}
void test202561(Body* body, std::string filePath);
Body* test202541(std::string filePath);

void test202513() {
    AsdkBody* ent1 = (AsdkBody*)getAsdkBody();
    AsdkBody* ent2 = (AsdkBody*)getAsdkBody();
    if (!ent1 || !ent2) {
        return;
    }
    body a, b;
    BodyTobody(&(ent1->body()), a);
    BodyTobody(&(ent2->body()), b);
    ent1->close();
    ent2->close();
    AinB.setLogicalLength(0); AoutB.setLogicalLength(0); BinA.setLogicalLength(0); BoutA.setLogicalLength(0);
    body2 ret;
    bodyBodyBool(a, b, Union, ret);
    AsdkBody* ent = new AsdkBody();
    bodyToBody(&ret, ent->body());
    {
        std::string path1("C:\\新建文件夹\\inputs\\inputs\\spod.obj");
        test202561(&ent->body(), path1);
    }
    AcDbObjectId id0;
    addToModelSpace(id0, ent);
}

void test202561(Body* body, std::string filePath)
{
    AcGePoint3dArray pts;
    std::vector<std::vector<int>> faces;
    std::unordered_map<Vertex*, int> vertexMap;
    int vertextIndex = 1;
    for (Vertex* v = body->vertexList(); v; v = v->next())
    {
        pts.append(v->point());
        vertexMap[v] = vertextIndex++;
    }
    for (Face* f = body->faceList(); f != NULL; f = f->next()) {
        std::vector<int> faceindex;
        Edge* curEdge = f->edgeLoop();
        do {
            Vertex* v = curEdge->vertex();
            faceindex.push_back(vertexMap[v]);
            curEdge = curEdge->next();
        } while (curEdge != f->edgeLoop());
        faces.push_back(faceindex);
    }
    std::ofstream file(filePath);
    if (!file.is_open())
    {
        return;
    }
    for (auto& v : pts) {
        file << "v " << v.x << " " << v.y << " " << v.z << "\n";
    }
    for (auto& f : faces) {
        file << "f";
        for (int& index : f) {
            file << " " << index;
        }
        file << "\n";
    }
    file.close();
}

#include <sstream>

Body* test202541(std::string filePath)
{
    std::ifstream file(filePath);
    if (!file.is_open())
    {
        return NULL;
    }
    AcGePoint3dArray pts;
    AcArray<Adesk::Int32> fs;
    std::string line;
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        std::string prefix;

        // 读取顶点  
        if (line.substr(0, 2) == "v ") {
            AcGePoint3d vertex;
            iss >> prefix >> vertex.x >> vertex.y >> vertex.z;
            pts.append(vertex);
        }
        // 读取面  
        else if (line.substr(0, 2) == "f ") {
            AcArray<Adesk::Int32> f;
            std::string vertexIndex;
            while (iss >> vertexIndex) {
                if (vertexIndex == "f") {
                    continue;
                }
                // OBJ 文件的索引是从 1 开始的，所以要减去 1  
                f.append(std::stoi(vertexIndex) - 1);
            }
            fs.append(f.length());
            fs.append(f);
        }
    }
    file.close();
    Body* body = new Body;
    std::vector<Vertex*> bodyVertex;
    for (auto p : pts)
    {
        const Point3d ptem(p.x, p.y, p.z);
        Vertex* vertex = new Vertex(ptem, body);
        bodyVertex.push_back(vertex);
    }

    std::map<noDirEdge, std::vector<Edge*>> edgeMap;

    for (int i = 0; i < fs.length(); i++)
    {
        int tolLength = fs[i];
        if (tolLength > 0)
        {
            i++;
            int start = fs[i];
            Face* face = new Face(body);
            Edge* firstEdge = nullptr;
            Edge* prevEdge = nullptr;
            for (int j = 0; j < tolLength; j++, i++)
            {
                int startIndex = fs[i];
                int endIndex;
                if (j == tolLength - 1) {
                    endIndex = start;
                }
                else {
                    endIndex = fs[i + 1];
                }
                Vertex* startVertex = bodyVertex[startIndex];
                Vertex* endVertex = bodyVertex[endIndex];
                noDirEdge parterKey = noDirEdge(startIndex, endIndex);
                auto it = edgeMap.find(parterKey);

                Edge* edge = new Edge(startVertex, face, prevEdge, NULL);
                if (it != edgeMap.end())
                {

                    it->second.push_back(edge);
                }
                else
                {
                    std::vector<Edge*> edges;
                    edges.push_back(edge);
                    edgeMap[parterKey] = edges;
                }
                if (!firstEdge) {
                    firstEdge = edge;
                }
                else {
                    prevEdge->setNext(edge);
                }
                prevEdge = edge;
            }
            i--;
            prevEdge->setNext(firstEdge);
            firstEdge->setPrev(prevEdge);
            face->setEdgeLoop(firstEdge);
        }
    }
    for (auto it : edgeMap)
    {
        std::vector<Edge*> t = it.second;
        if (t.size() == 2)
        {
            if (t[0]->vertex() == t[1]->vertex()) {
                continue;
            }
            t[0]->setPartner(t[1]);
            t[1]->setPartner(t[0]);
        }
        if (t.size() == 4)
        {
            std::vector<Edge*> t0;
            std::vector<Edge*> t1;
            t0.push_back(t[0]);
            for (int i = 1; i < 4; i++) {
                if (t[i]->vertex() == t[0]->vertex()) {
                    t0.push_back(t[i]);
                }
                else {
                    t1.push_back(t[i]);
                }
            }
            if (t0.size() != t1.size()) {
                continue;
            }
            t0[0]->setPartner(t1[0]);
            t1[0]->setPartner(t0[1]);
            t0[1]->setPartner(t1[1]);
            t1[1]->setPartner(t0[0]);

        }
        if (t.size() == 3)
        {
            int ttt = 234;
        }
    }
    return body;
}
