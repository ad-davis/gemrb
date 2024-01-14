/* GemRB - Infinity Engine Emulator
 * Copyright (C) 2023 The GemRB Project
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 *
 */

#include "OverHeadText.h"

#include "Game.h"
#include "Interface.h"
#include "Scriptable.h"

#include "GUI/GameControl.h"

namespace GemRB {

const String& OverHeadText::GetText(size_t idx) const
{
	if (idx >= messages.size()) return emptyString;
	return messages[idx].text;
}

static constexpr int maxScrollOffset = 100;
void OverHeadText::SetText(String newText, bool display, bool append, const Color& newColor, size_t idx)
{
	while (idx >= messages.size()) {
		messages.emplace_back();
	}
	if (newText.empty()) {
		messages[idx].pos.Invalidate();
		Display(false, idx);
		return;
	}

	// always append for actors and areas ... unless it's the head hp ratio
	if (append && core->HasFeature(GFFlags::ONSCREEN_TEXT) && (owner->Type == ST_ACTOR || owner->Type == ST_AREA)) {
		idx = messages.size();
		messages.emplace_back();
		if (owner->Type == ST_ACTOR) {
			messages[idx].scrollOffset = Point(0, maxScrollOffset);
		}
	} else {
		messages[idx].scrollOffset.Invalidate();
	}
	messages[idx].pos.Invalidate();
	messages[idx].text = std::move(newText);
	messages[idx].color = newColor;
	Display(display, idx);
}

static tick_t display_time(const OverHeadMsg& msg) {
	if (msg.timeStartDisplaying == 0) return 0;
	static constexpr tick_t maxDelay = 6000;
	tick_t delay = maxDelay;
	if (core->HasFeature(GFFlags::ONSCREEN_TEXT) && !msg.scrollOffset.IsInvalid()) {
		// empirically determined estimate to get the right speed and distance and 2px/tick
		delay = 1800;
	}
	tick_t time = core->Time.Ticks2Ms(core->GetGame()->GameTime);
	time -= msg.timeStartDisplaying;
	if (time >= delay) {
		return 0;
	} else {
		return (delay - time) / 10;
	}

}

bool OverHeadText::IsDisplaying() const {
	bool showing = false;
	for (const auto& msg : messages) {
		showing = showing || display_time(msg) != 0;
	}
	return showing;
}

bool OverHeadText::Display(bool show, size_t idx)
{
	if (idx >= messages.size()) {
		return false;
	}
	if (show) {
		messages[idx].timeStartDisplaying = core->Time.Ticks2Ms(core->GetGame()->GameTime);
		return true;
	} else {
		// is this the last displaying message?
		if (messages.size() == 1 && idx == 0) {
			messages[idx].timeStartDisplaying = 0;
			return true;
		} else if (idx < messages.size()) {
			messages.erase(messages.begin() + idx);
			return true;
		}
	}
	return false;
}

// 'fix' the current overhead text - follow owner's position
void OverHeadText::FixPos(const Point& pos, size_t idx)
{
	if (idx >= messages.size()) return;
	messages[idx].pos = pos;
}

int OverHeadText::GetHeightOffset() const
{
	int offset = 100;
	if (owner->Type == ST_ACTOR) {
		offset = static_cast<const Selectable*>(owner)->circleSize * 30;
	}

	return offset;
}

void OverHeadText::Draw()
{
	int height = GetHeightOffset();
	// start displaying from first message's point
	const Point p = messages[0].pos.IsInvalid() ? owner->Pos : messages[0].pos;
	for (auto msgIter = messages.begin(); msgIter != messages.end(); ++msgIter) {
		auto& msg = *msgIter;
		bool drawn = msg.Draw(height, p, owner->Type);
		if (!drawn && msgIter != messages.begin()) { // always keep the one reserved slot
			msgIter = messages.erase(msgIter);
			--msgIter;
		}
	}
}

// *******************************************************************
// OverHeadMsg methods
bool OverHeadMsg::Draw(int& heightOffset, const Point& startPos, int ownerType)
{
	if (text.empty()) return false;

	Color& textColor = color;
	if (color == ColorBlack) {
		// use defaults
		if (ownerType == ST_ACTOR) {
			textColor = displaymsg->GetColor(GUIColors::FLOAT_TXT_ACTOR);
		} else if (ownerType == ST_TRIGGER) {
			textColor = displaymsg->GetColor(GUIColors::FLOAT_TXT_INFO);
		} else {
			textColor = displaymsg->GetColor(GUIColors::FLOAT_TXT_OTHER);
		}
	}
	Font::PrintColors fontColor = { textColor, ColorBlack };

	tick_t time = display_time(*this);
	if (time == 0) {
		timeStartDisplaying = 0;
		return false;
	} else if (time < 256) {
		// rapid fade-out
		fontColor.fg.a = static_cast<unsigned char>(255 - time);
	}

	Font* font = core->GetTextFont();

	// in practice things can overflow the height
	Size textboxSize(200, 400);

	// calculate the height of the resulting string and add it to the offset 
	Font::StringSizeMetrics metrics = {Size(textboxSize), 0, 0, true};
	Size stringSize = font->StringSize(text, &metrics);
	heightOffset += stringSize.h;

	Region vp = core->GetGameControl()->Viewport();
	Region rgn(startPos - Point(100, heightOffset) - vp.origin, textboxSize);
	if (core->HasFeature(GFFlags::ONSCREEN_TEXT) && !scrollOffset.IsInvalid()) {
		rgn.y -= maxScrollOffset - scrollOffset.y;
		// rgn.h will be adjusted automatically, we don't need to worry about accidentally hiding other msgs
		scrollOffset.y -= 2;
	}
	font->Print(rgn, text, IE_FONT_ALIGN_CENTER | IE_FONT_ALIGN_TOP, fontColor);

	return true;
}

}
