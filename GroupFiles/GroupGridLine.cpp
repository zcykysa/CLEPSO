#include "GroupGridLine.h"
#include "vtkPolyDataMapper.h"
#include "vtkPolyData.h"
#include "vtkRenderer.h"
#include "vtkRenderWindow.h"
#include "vtkActor.h"
#include "vtkProperty.h"
#include "OrthoData.h"
#include "QSharedPointer.h"
#include "Arch.h"
#include "Common.h"
#include "GroupBase.h"
#include "RendererContainer.h"
#include <QColor>
#include <QPalette>
#include <QPointF>

GroupGridLine::GroupGridLine(GroupType type) : GroupBase(type) {
	m_GroupType = type;
    LinePoints = vtkSmartPointer<vtkPoints>::New();
    LineCellArray = vtkSmartPointer<vtkCellArray>::New();
    LinePolyData = vtkSmartPointer<vtkPolyData>::New();
    LinePolyMapper = vtkSmartPointer<vtkDataSetMapper>::New();
    ToothRenderer = vtkSmartPointer<vtkRenderer>::New();
    LineRenderer = vtkSmartPointer<vtkRenderer>::New();
    
}

GroupGridLine::~GroupGridLine() {
}


void GroupGridLine::OnGridLine() {
    m_RenderContainer->GetRenderWindow()->RemoveRenderer(m_RenderContainer->GetRenderer(MainRenderer));
    ToothRenderer->SetBackground(0.717, 0.730, 0.742);
    LineRenderer->SetBackground(0.717, 0.730, 0.742);
    //绘制网格方法一
    //// 平行Y方向的直线
    //for (double gridX = -100; gridX < 100; gridX += 10)
    //{
    //    double lineStart[3] = { gridX, -100.0, 0.0 };
    //    double lineEnd[3] = { gridX, 100.0, 0.0 };

    //    vtkIdType pointIdStart = LinePoints->InsertNextPoint(lineStart);
    //    vtkIdType pointIdEnd = LinePoints->InsertNextPoint(lineEnd);

    //    vtkIdType singleLineCell[2] = { pointIdStart, pointIdEnd };
    //    LineCellArray->InsertNextCell(2, singleLineCell);
    //}
    //// 平行X方向的直线
    //for (double gridY = -100; gridY < 100; gridY += 10)
    //{
    //    double lineStart[3] = { -100.0, gridY, 0.0 };
    //    double lineEnd[3] = { 100.0, gridY, 0.0 };

    //    vtkIdType pointIdStart = LinePoints->InsertNextPoint(lineStart);
    //    vtkIdType pointIdEnd = LinePoints->InsertNextPoint(lineEnd);

    //    vtkIdType singleLineCell[2] = { pointIdStart, pointIdEnd };
    //    LineCellArray->InsertNextCell(2, singleLineCell);
    ////}
    //LinePolyData->SetLines(LineCellArray);
    //LinePolyData->SetPoints(LinePoints);
    //LinePolyMapper->SetInputData(LinePolyData);

    //vtkSmartPointer<vtkActor>  line =
    //    vtkSmartPointer<vtkActor>::New();
    //line->SetMapper(LinePolyMapper);
    //LineRenderer->AddActor(line);
    
    //绘制网格方法二
    //绘制水平线
    for (float row = 0; row <= 1.05; row += 0.05)
    {
        double p0[3] = { row, 0, 0 };
        double p1[3] = { row, 1, 0 };
        vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
        points->InsertNextPoint(p0);
        points->InsertNextPoint(p1);
        vtkSmartPointer<vtkLine> pLine = vtkSmartPointer<vtkLine>::New();
        pLine->GetPointIds()->SetId(0, 0);
        pLine->GetPointIds()->SetId(1, 1);
        vtkSmartPointer<vtkCellArray> cellArray = vtkSmartPointer<vtkCellArray>::New();
        cellArray->InsertNextCell(pLine);

        vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
        polyData->SetPoints(points);
        polyData->SetLines(cellArray);

        vtkSmartPointer<vtkPolyDataMapper> mapper =
            vtkSmartPointer<vtkPolyDataMapper>::New();
        mapper->SetInputData(polyData);

        vtkSmartPointer<vtkActor> lineActor =
            vtkSmartPointer<vtkActor>::New();
        lineActor->SetMapper(mapper);
        lineActor->GetProperty()->SetColor(1, 1, 1);
        lineActor->GetProperty()->SetLineWidth(1);
        LineRenderer->AddActor(lineActor);
    }
    //绘制垂直线
    for (float col = 0; col <= 1.05; col += 0.05)
    {
        double p0[3] = { 0, col, 0 };
        double p1[3] = { 1, col, 0 };
        vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
        points->InsertNextPoint(p0);
        points->InsertNextPoint(p1);
        vtkSmartPointer<vtkLine> pLine = vtkSmartPointer<vtkLine>::New();
        pLine->GetPointIds()->SetId(0, 0);
        pLine->GetPointIds()->SetId(1, 1);
        vtkSmartPointer<vtkCellArray> cellArray = vtkSmartPointer<vtkCellArray>::New();
        cellArray->InsertNextCell(pLine);

        vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
        polyData->SetPoints(points);
        polyData->SetLines(cellArray);

        vtkSmartPointer<vtkPolyDataMapper> mapper =
            vtkSmartPointer<vtkPolyDataMapper>::New();
        mapper->SetInputData(polyData);

        vtkSmartPointer<vtkActor> lineActor =
            vtkSmartPointer<vtkActor>::New();
        lineActor->SetMapper(mapper);
        lineActor->GetProperty()->SetColor(1, 1, 1);
        lineActor->GetProperty()->SetLineWidth(1);
        LineRenderer->AddActor(lineActor);
    }
    

    for (int i = 1; i <= 32; i++) {
        Tooth* tooth = m_OrthoData->GetTooth((ToothId)i);
        if (tooth == NULL) continue;
        tooth->GetToothActor()->SetVisibility(true);
        ToothRenderer->AddActor(tooth->GetToothActor());//将牙齿Actor添加至渲染器中
    }

    
    m_RenderContainer->GetRenderWindow()->SetNumberOfLayers(2);
    ToothRenderer->SetLayer(1);
    LineRenderer->SetLayer(0);
    m_RenderContainer->GetRenderWindow()->AddRenderer(LineRenderer);
    m_RenderContainer->GetRenderWindow()->AddRenderer(ToothRenderer);

	m_RenderContainer->Render();
}





void GroupGridLine::OpenGroup() {
	m_IsOpen = true;
	cout << "打开功能：网格" << endl;
}

void GroupGridLine::CloseGroup() {
	m_IsOpen = false;
    m_RenderContainer->GetRenderWindow()->RemoveRenderer(LineRenderer);
    m_RenderContainer->GetRenderWindow()->RemoveRenderer(ToothRenderer);
    m_RenderContainer->GetRenderWindow()->AddRenderer(m_RenderContainer->GetRenderer(MainRenderer));
    cout << "关闭功能：网格" << endl;
	m_RenderContainer->Render();
}