#ifndef GUI_DIAGRAM_HPP
#define GUI_DIAGRAM_HPP

// Qt includes
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QImage>
#include <QObject>

namespace robot_gui {

	class GuiDiagram : public QGraphicsScene {

		Q_OBJECT

		public:

			enum Mode { InsertItem, InsertLine, InsertText, MoveItem };

			explicit DiagramScene(QMenu *itemMenu, QObject *parent = 0);

			QFont font() const { return myFont; }
			QColor textColor() const { return myTextColor; }
			QColor itemColor() const { return myItemColor; }
			QColor lineColor() const { return myLineColor; }
			void setLineColor(const QColor &color);
			void setTextColor(const QColor &color);
			void setItemColor(const QColor &color);
			void setFont(const QFont &font);

		public slots:

			void setMode(Mode mode);
			void setItemType(DiagramItem::DiagramType type);
			void editorLostFocus(DiagramTextItem *item);

		signals:

			void itemInserted(DiagramItem *item);
			void textInserted(QGraphicsTextItem *item);
			void itemSelected(QGraphicsItem *item);

		private:

			bool isItemChange(int type);

			DiagramItem::DiagramType myItemType;
			QMenu *myItemMenu;
			Mode mode_;
			bool leftButtonDown;
			QPointF startPoint;
			QGraphicsLineItem* line;
			QFont font_;
			GuiDiagramText* diagram_text_;
			QColor text_color_;
			QColor item_color_;
			QColor line_color_;
	};
}

#endif  // GUI_DISPLAY_CAMERA_HPP
