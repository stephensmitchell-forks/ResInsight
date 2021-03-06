/////////////////////////////////////////////////////////////////////////////////
//
//  Copyright (C) 2019-     Equinor ASA
//
//  ResInsight is free software: you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation, either version 3 of the License, or
//  (at your option) any later version.
//
//  ResInsight is distributed in the hope that it will be useful, but WITHOUT ANY
//  WARRANTY; without even the implied warranty of MERCHANTABILITY or
//  FITNESS FOR A PARTICULAR PURPOSE.
//
//  See the GNU General Public License at <http://www.gnu.org/licenses/gpl.html>
//  for more details.
//
//////////////////////////////////////////////////////////////////////////////////
#include "RiaGrpcServiceInterface.h"

#include "RiaApplication.h"
#include "RimCase.h"
#include "RimProject.h"

#include "RicfFieldHandle.h"
#include "RicfMessages.h"

#include "cafPdmChildArrayField.h"
#include "cafPdmChildField.h"
#include "cafPdmDataValueField.h"
#include "cafPdmObject.h"
#include "cafPdmXmlFieldHandle.h"

#include <grpcpp/grpcpp.h>

#include <PdmObject.pb.h>
#include <QXmlStreamReader>
//--------------------------------------------------------------------------------------------------
///
//--------------------------------------------------------------------------------------------------
RimCase* RiaGrpcServiceInterface::findCase( int caseId )
{
    std::vector<RimCase*> cases;
    RiaApplication::instance()->project()->allCases( cases );

    for ( RimCase* rimCase : cases )
    {
        if ( caseId == rimCase->caseId() )
        {
            return rimCase;
        }
    }
    return nullptr;
}

//--------------------------------------------------------------------------------------------------
/// Find the number of messages that will fit in the given bytes.
/// The default argument is meant to be a sensible size for GRPC.
//--------------------------------------------------------------------------------------------------
size_t RiaGrpcServiceInterface::numberOfMessagesForByteCount( size_t messageSize,
                                                              size_t numBytesWantedInPackage /*= 64 * 1024u*/ )
{
    size_t messageCount = numBytesWantedInPackage / messageSize;
    return messageCount;
}

//--------------------------------------------------------------------------------------------------
///
//--------------------------------------------------------------------------------------------------
void RiaGrpcServiceInterface::copyPdmObjectFromCafToRips( const caf::PdmObjectHandle* source, rips::PdmObject* destination )
{
    CAF_ASSERT( source && destination && source->xmlCapability() );

    destination->set_class_keyword( source->xmlCapability()->classKeyword().toStdString() );
    destination->set_address( reinterpret_cast<uint64_t>( source ) );

    bool visible = true;
    if ( source->uiCapability() && source->uiCapability()->objectToggleField() )
    {
        const caf::PdmField<bool>* boolField =
            dynamic_cast<const caf::PdmField<bool>*>( source->uiCapability()->objectToggleField() );
        if ( boolField )
        {
            visible = boolField->value();
        }
    }
    destination->set_visible( visible );

    std::vector<caf::PdmFieldHandle*> fields;
    source->fields( fields );

    auto parametersMap = destination->mutable_parameters();
    for ( auto field : fields )
    {
        auto pdmValueField = dynamic_cast<const caf::PdmValueField*>( field );
        if ( pdmValueField )
        {
            QString keyword    = pdmValueField->keyword();
            auto    ricfHandle = field->template capability<RicfFieldHandle>();
            if ( ricfHandle != nullptr )
            {
                QString     text;
                QTextStream outStream( &text );
                ricfHandle->writeFieldData( outStream, false );
                ( *parametersMap )[keyword.toStdString()] = text.toStdString();
            }
        }
    }
}

//--------------------------------------------------------------------------------------------------
///
//--------------------------------------------------------------------------------------------------
void RiaGrpcServiceInterface::copyPdmObjectFromRipsToCaf( const rips::PdmObject* source, caf::PdmObjectHandle* destination )
{
    CAF_ASSERT( source && destination && destination->xmlCapability() );
    CAF_ASSERT( source->class_keyword() == destination->xmlCapability()->classKeyword().toStdString() );

    if ( destination->uiCapability() && destination->uiCapability()->objectToggleField() )
    {
        caf::PdmField<bool>* boolField =
            dynamic_cast<caf::PdmField<bool>*>( destination->uiCapability()->objectToggleField() );
        if ( boolField )
        {
            QVariant oldValue = boolField->toQVariant();
            boolField->setValue( source->visible() );
            QVariant newValue = boolField->toQVariant();
            destination->uiCapability()->fieldChangedByUi( boolField, oldValue, newValue );
        }
    }

    std::vector<caf::PdmFieldHandle*> fields;
    destination->fields( fields );

    auto parametersMap = source->parameters();
    for ( auto field : fields )
    {
        auto pdmValueField = dynamic_cast<caf::PdmValueField*>( field );
        if ( pdmValueField )
        {
            QString keyword = pdmValueField->keyword();
            QString value   = QString::fromStdString( parametersMap[keyword.toStdString()] );

            QVariant oldValue, newValue;
            if ( assignFieldValue( value, pdmValueField, &oldValue, &newValue ) )
            {
                destination->uiCapability()->fieldChangedByUi( field, oldValue, newValue );
            }
        }
    }
}

//--------------------------------------------------------------------------------------------------
///
//--------------------------------------------------------------------------------------------------
bool RiaGrpcServiceInterface::assignFieldValue( const QString&      stringValue,
                                                caf::PdmValueField* field,
                                                QVariant*           oldValue,
                                                QVariant*           newValue )
{
    CAF_ASSERT( oldValue && newValue );

    auto ricfHandle = field->template capability<RicfFieldHandle>();
    if ( field && ricfHandle != nullptr )
    {
        QTextStream  stream( stringValue.toLatin1() );
        RicfMessages messages;
        *oldValue = field->toQVariant();
        ricfHandle->readFieldData( stream, nullptr, &messages, false );
        *newValue = field->toQVariant();
        return true;
    }
    return false;
}

//--------------------------------------------------------------------------------------------------
///
//--------------------------------------------------------------------------------------------------
caf::PdmObjectHandle* RiaGrpcServiceInterface::emplaceChildField( caf::PdmObject* parent, const QString& fieldLabel )
{
    std::vector<caf::PdmFieldHandle*> fields;
    parent->fields( fields );

    for ( auto field : fields )
    {
        auto pdmChildArrayField = dynamic_cast<caf::PdmChildArrayFieldHandle*>( field );
        auto pdmChildField      = dynamic_cast<caf::PdmChildFieldHandle*>( field );
        if ( pdmChildArrayField && pdmChildArrayField->keyword() == fieldLabel )
        {
            return emplaceChildArrayField( pdmChildArrayField );
        }
        else if ( pdmChildField && pdmChildField->keyword() == fieldLabel )
        {
            return emplaceChildField( pdmChildField );
        }
    }
    return nullptr;
}

//--------------------------------------------------------------------------------------------------
///
//--------------------------------------------------------------------------------------------------
caf::PdmObjectHandle* RiaGrpcServiceInterface::emplaceChildField( caf::PdmChildFieldHandle* childField )
{
    QString childClassKeyword = childField->xmlCapability()->childClassKeyword();

    auto pdmObjectHandle = caf::PdmDefaultObjectFactory::instance()->create( childClassKeyword );
    CAF_ASSERT( pdmObjectHandle );
    childField->setChildObject( pdmObjectHandle );
    return pdmObjectHandle;
}

//--------------------------------------------------------------------------------------------------
///
//--------------------------------------------------------------------------------------------------
caf::PdmObjectHandle* RiaGrpcServiceInterface::emplaceChildArrayField( caf::PdmChildArrayFieldHandle* childArrayField )
{
    QString childClassKeyword = childArrayField->xmlCapability()->childClassKeyword();

    auto pdmObjectHandle = caf::PdmDefaultObjectFactory::instance()->create( childClassKeyword );
    CAF_ASSERT( pdmObjectHandle );

    childArrayField->insertAt( -1, pdmObjectHandle );
    return pdmObjectHandle;
}
